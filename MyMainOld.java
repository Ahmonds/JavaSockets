package LIFT;

import java.net.*;
import java.io.*;
import java.sql.*;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

import javax.inject.Inject;
import com.kuka.graph.Graph;
import com.kuka.common.ThreadUtil;
import com.kuka.resource.locking.LockException;
import com.kuka.task.ITask;
import com.kuka.task.ITaskLogger;
import com.kuka.task.ITaskManager;
import com.kuka.task.RoboticsAPITask;
import com.kuka.task.properties.ParameterSetter;
import com.kuka.roboticsAPI.controllerModel.sunrise.state.kmp.io.ScannerIOGroup;

import com.kuka.nav.Location;
import com.kuka.nav.OrientationMode;
import com.kuka.nav.Orientations;
import com.kuka.nav.Pose;
import com.kuka.nav.Position;
import com.kuka.nav.XYTheta;
import com.kuka.nav.data.LocationData;
import com.kuka.nav.fineloc.FineLocalizationContainer;
import com.kuka.nav.fineloc.FineLocalizationRequest;
import com.kuka.nav.fineloc.ObjectLocalizationContainer;
import com.kuka.nav.fineloc.ObjectLocalizationRequest;
import com.kuka.nav.fleet.BaseGraphMotionListener;
import com.kuka.nav.fleet.ChangeGraphCommand;
import com.kuka.nav.fleet.Conflict;
import com.kuka.nav.fleet.ConflictFilter;
import com.kuka.nav.fleet.CoordinatedGraphMotionContainer;
import com.kuka.nav.fleet.CoordinatedGraphMotionListener;
import com.kuka.nav.fleet.CorrectPlansCallback;
import com.kuka.nav.fleet.FleetManager;
import com.kuka.nav.fleet.GraphMotion;
import com.kuka.nav.fleet.GraphMotionContainer;
import com.kuka.nav.fleet.GraphMotionContainerBase;
import com.kuka.nav.fleet.GraphMotionListener;
import com.kuka.nav.fleet.RemoveFromGraphCommand;
import com.kuka.nav.fleet.ReplanResult;
import com.kuka.nav.fleet.actions.CustomEdgeAction;
import com.kuka.nav.fleet.actions.CustomNodeAction;
import com.kuka.nav.fleet.filter.InstanceFilter;
import com.kuka.nav.fleet.graph.GraphData;
import com.kuka.nav.fleet.graph.TopologyGraph;
import com.kuka.nav.fleet.graph.TopologyNode;
import com.kuka.nav.geometry.DetectionModel;
import com.kuka.nav.line.VirtualLine;
import com.kuka.nav.line.VirtualLineMotion;
import com.kuka.nav.object.filter.Restriction;
import com.kuka.nav.object.filter.SearchSpace;
import com.kuka.nav.object.handling.ObjectDetection;
import com.kuka.nav.object.handling.ObjectDetectionContainer;
import com.kuka.nav.object.handling.ObjectDetectionRequest;
import com.kuka.nav.object.handling.TrackingMotion;
import com.kuka.nav.recover.OffPathCondition;
import com.kuka.nav.recover.OffStartPoseCondition;
import com.kuka.nav.recover.ReturnToPathRecover;
import com.kuka.nav.recover.ReturnToStartPoseRecover;
import com.kuka.nav.rel.RelativeMotion;
import com.kuka.nav.robot.ChangeMapCommand;
import com.kuka.nav.robot.LocalizeCommand;
import com.kuka.nav.robot.MobileRobot;
import com.kuka.nav.robot.MobileRobotManager;
import com.kuka.nav.task.NavTaskCategory;
import com.kuka.nav.task.remote.RemoteTaskId;
import com.kuka.nav.task.remote.TaskRequest;
import com.kuka.nav.task.remote.TaskRequestContainer;

import examples.Ex_01_RelativeMotion;

//@NavTaskCategory(maxInstances=2, autoStart=true)
@NavTaskCategory(autoStart=true) // Set to false for manual start in map perspective 
public class MyMain extends RoboticsAPITask {

	@Inject
	private MobileRobotManager _robMan;
	private MobileRobot _rob;
	@Inject
	private ITaskLogger _log;
	@Inject
	private ITaskManager _taskMan;
	private int _robotId;
	private LocationData _locDat;
	@Inject
	private GraphData _graphDat;
	@Inject
	private FleetManager _fleetMan;
	
	//	PostgreSQL
	private Connection conn = null;
	private Statement stmt = null;
	private PreparedStatement prepSt = null;
	CallableStatement cstmt = null;
	private ResultSet rs = null;
	final String url = "jdbc:postgresql://localhost:5432/kmrdb";
	final String username = "postgres", pw = "kuka"; 
	
    //	Request Postgres Storage Data
	final byte numB = 6;
	private byte binSrc = 0, binDest = 0, boxSrc = 0, boxDest = 0, stat = 0;
	private byte[] BinL = new byte[numB], BinQ = new byte[numB], BoxL = new byte[numB];
	private String binInfo = "", boxInfo = "";
	
	KMRstorage kStore = new KMRstorage();
	
	private int targetStn = 0, startStn = 0;
	/*@ParameterSetter
	public void setJogKMR (Integer StnFrom, Integer StnTo){
		
		startStn = StnFrom.intValue();
		targetStn = StnTo.intValue();
	}*/
	
	@Override
	public void initialize() throws Exception {
		Thread.sleep(500);
		_robotId = 1;
		_rob = _robMan.getRobot(_robotId);
		_log.info("Initialize finished.");
	}

	//Move to Location
	public void executeMotionToLoc() {
		
		Location Loc = _locDat.get(LocID);
		XYTheta VelOverride = (new XYTheta (0.1, 0.1, Math.toRadians(10)));
		
		//Velocity parameters for X,Y and Theta
		VirtualLineMotion viLineMoIni = new VirtualLineMotion(_rob.getPose(), Loc.getPose());
		_rob.move(viLineMoIni.setVelocityOverride(VelOverride));	
	}
	
	//Execute graph motion to station
	public void executeGraphMo() {

		int graphId = 1;
		TopologyGraph testGraph = _graphDat.get(graphId);
		TopologyNode startNode = testGraph.getNode(StartNode);
		TopologyNode targetNode = testGraph.getNode(TargetNode);
		XYTheta VelOverride = (new XYTheta (0.5, 0.5, Math.toRadians(10)));
		
		//Virtual line motion from KMR current position to start node
		VirtualLineMotion moveToGraph = new VirtualLineMotion(_rob.getPose(), startNode.getPosition().toPose(0));
		_rob.move(moveToGraph.setVelocityOverride(VelOverride));
		
		//set the robot to the graph
		ChangeGraphCommand graphCom = new ChangeGraphCommand(testGraph.getId(), startNode.getId());
		_rob.execute(graphCom);
		_rob.unlock();
		
		//move on graph
		GraphMotion graphMo = new GraphMotion(testGraph, targetNode);
		InstanceFilter filter = new InstanceFilter(_robotId);
		Collection<MobileRobot> all = _robMan.getAll(MobileRobot.class);
		for (Iterator iterator = all.iterator(); iterator.hasNext();) {
			MobileRobot mobileRobot = (MobileRobot) iterator.next();
			if(mobileRobot.getBatteryState() > 0.3){
				_robotId = mobileRobot.getId();
				break;
			}
		}
		graphMo.setResourceFilter(filter);
		
		_fleetMan.execute(graphMo);	
	}
	
	// Don't change these variables directly
	private int LocID = 0, TargetNode = 0, StartNode = 0;
	public void ReadTargetNode() {
	    switch (targetStn) {
	        case 1:
	        	LocID = 16;			//Station 10
	        	TargetNode = 8;
	            break;
	        case 2:
	        	LocID = 15;			//Station 20
		  		TargetNode = 7;
		  		break; 
	        case 3:
	        	LocID = 14;			//Station 30
		  		TargetNode = 6;
		  		break;
	        case 4:
	        	LocID = 13;			//Station 40
		  		TargetNode = 5;
		  		break;
	        case 5:
	        	LocID = 12;			//Station 50
		  		TargetNode = 4;
		  		break; 
	        case 6:
	        	LocID = 11;			//Station 60
		  		TargetNode = 3;
		  		break;
	        case 7:
	        	LocID = 10;			//Station 70
		  		TargetNode = 2;
		  		break;
		    default:
		    	LocID = 9;			//Load rack
		    	TargetNode = 1;
	    }
	}
	public void ReadStartNode() {
	    switch (startStn) {
	        case 1:
	        	StartNode = 8;			//Station 10
	            break;
	        case 2:
	        	StartNode = 7;			//Station 20
		  		break;                    	
	        case 3:
	        	StartNode = 6;			//Station 30 
		  		break; 	  			  
	        case 4:
	        	StartNode = 5;			//Station 40
		  		break; 
	        case 5:
	        	StartNode = 4;			//Station 50 
		  		break; 
	        case 6:
	        	StartNode = 3;			//Station 60 
		  		break; 	  			  
	        case 7:
	        	StartNode = 2;			//Station 70
		  		break;
		    default:
		    	StartNode = 1;			//Load rack
	    }
	}
	
	public class remotePars implements Serializable {
		private static final long serialVersionUID = 1L;
		public int iDat = 0;
		public String sDat = "Test";
	}
	remotePars par = new remotePars();
	
	private void executeLocalTask() {
		//ITask taskToCall = _taskMan.getTask(LocalTaskExample.class);
		
		ITask taskToCall = _taskMan.getTask(Ex_01_RelativeMotion.class); // <---
		//										^	^	^	^	put file name of "local task" to be executed here
		
		try {
			taskToCall.startInstance();
		} catch (InterruptedException e) {}
		ThreadUtil.milliSleep(3000);
		_log.info("Local task complete");
	}

	private void executeRemoteTask(remotePars p) {	
		ObjectOutputStream out = null;
		try {
			FileOutputStream fos = new FileOutputStream("/tmp/iiwaPar.ser");
			out = new ObjectOutputStream(fos);
			out.writeObject(p);
		} catch (IOException e) {}
		
		String s = "application.MoveToDrivePos";
		RemoteTaskId remoTaskId = new RemoteTaskId(s);
// put file name of "remote task" to be executed here  ^	^	^	^	^	^
		TaskRequest request = new TaskRequest(remoTaskId, "/tmp/iiwaPar.ser");
		//TaskRequest request = new TaskRequest(remoTaskId);
		
		TaskRequestContainer container = _rob.execute(request);
		container.awaitFinished();
		
		_log.info("Rmote task complete");
	}

	@SuppressWarnings("unused")
	private void createStorage() throws SQLException {
		
		stmt.executeUpdate("drop table if exists kstore");
		stmt.executeUpdate("CREATE TABLE kstore " +
			   	"(id INT PRIMARY KEY		NOT NULL," +
			    
			   	" BinID			INTEGER		NOT NULL," +
			   	" BinQuantity	SMALLINT	NOT NULL," +
			   	" BinName		CHAR(10)," +
			   	
			   	" BoxID			INTEGER		NOT NULL," +
		   	   	" BoxName		CHAR(10)," +
			   	" BoxBit		BOOLEAN		NOT NULL)");
		
		_log.info("Storage Table Created\n");
		
		prepSt = conn.prepareStatement("INSERT INTO kstore " +
				"(id, BinID, BinQuantity, BinName, BoxID, BoxName, BoxBit)" +
				"VALUES (1, 0, 0, '', 0, '', false), (2, 0, 0, '', 0, '', false)," +
				"(3, 0, 0, '', 0, '', false), (4, 0, 0, '', 0, '', false)");
		
		prepSt.executeUpdate();
		
		_log.info("Insert to kStore Complete");
	}
	/*
	@SuppressWarnings("unused")
	private void updateKStorDat(int id, String bb) {
		
		if(bb.toLowerCase().equals("bin")) {
			try {
				prepSt = conn.prepareStatement("UPDATE kstore SET "+
						"BinID = ?, BinQuantity = ?, BinName = ? WHERE id = ?;");
				prepSt.setInt(1, kStore.binID[id-1]);
				prepSt.setByte(2, kStore.binQnty[id-1]);
				prepSt.setString(3, kStore.binName[id-1]);
				prepSt.setInt(4, id);
				
			} catch (SQLException e) { }
		}
		else if(bb.toLowerCase().equals("box")) {
			try {
				prepSt = conn.prepareStatement("UPDATE kstore SET "+
						"BoxID = ?, BoxName = ?, BoxBit = ? WHERE id = ?;");
				prepSt.setInt(1, kStore.boxID[id-1]);
				prepSt.setString(2, kStore.boxName[id-1]);
				prepSt.setBoolean(3, kStore.boxBit[id-1]);
				prepSt.setInt(4, id);
				
			} catch (SQLException e) { }
		}
		
		try {
			prepSt.executeUpdate();
			_log.info("Position Update of kStore Complete");
		} catch (SQLException e) { }
	}
	*/
	@SuppressWarnings("unused")
	private void updateStorDat(int id, String col, int d) {
		
		try {
			prepSt = conn.prepareStatement("UPDATE kstore SET " + col + " = ? WHERE id = ?");
			prepSt.setInt(1, d);
			prepSt.setInt(2, id);
			
			prepSt.executeUpdate();
			_log.info("Integer Update of kStore Complete");
			
		} catch (SQLException e) { _log.error("Integer Update of kStore Failed");}
	}
	@SuppressWarnings("unused")
	private void updateStorDat(int id, String col, byte d) {
		
		try {
			prepSt = conn.prepareStatement("UPDATE kstore SET " + col + " = ? WHERE id = ?");
			prepSt.setByte(1, d);
			prepSt.setInt(2, id);
			
			prepSt.executeUpdate();
			_log.info("Byte Update of kStore Complete");
			
		} catch (SQLException e) { _log.error("Byte Update of kStore Failed"); }
	}
	@SuppressWarnings("unused")
	private void updateStorDat(int id, String col, String d) {
		
		try {
			prepSt = conn.prepareStatement("UPDATE kstore SET " + col + " = '" + d + "' WHERE id = ?");
			prepSt.setInt(1, id);
			
			prepSt.executeUpdate();
			_log.info("String Update of kStore Complete");
			
		} catch (SQLException e) { _log.error("String Update of kStore Failed"); }
	}
	@SuppressWarnings("unused")
	private void updateStorDat(int id, String col, Boolean d) {
		
		try {
			prepSt = conn.prepareStatement("UPDATE kstore SET " + col + " = ? WHERE id = ?");
			prepSt.setBoolean(1, d);
			prepSt.setInt(2, id);
			
			prepSt.executeUpdate();
			_log.info("Boolean Update of kStore Complete");
			
		} catch (SQLException e) {_log.error("Boolean Update of kStore Failed");}
	}
	
	@SuppressWarnings({"unused", "unchecked"})
	private <T> T readKStoreDat(int id, String col) throws SQLException {
		
	    prepSt = conn.prepareStatement("SELECT ? FROM kstore WHERE id = ?");
	    prepSt.setString(1, col);
	    prepSt.setInt(2, id);
	    rs = prepSt.executeQuery();
	    
	    rs.next();
	    
	    if(rs.getMetaData().getColumnTypeName(1).toLowerCase().equals("integer")) return (T) new Integer(rs.getInt(1));
	    else if(rs.getMetaData().getColumnTypeName(1).toLowerCase().equals("smallint")) return (T) new Byte(rs.getByte(1));
	    else if(rs.getMetaData().getColumnTypeName(1).toLowerCase().equals("character()")) return (T) new String(rs.getString(1));
	    else if(rs.getMetaData().getColumnTypeName(1).toLowerCase().equals("boolean")) return (T) new Boolean(rs.getBoolean(1));
	    else return null;
	}
	
	private void callKStoreRead(int id) {
		try {
			//prepSt = conn.prepareStatement("SELECT fncget_kmr_inventory(id => ?)");
			cstmt = conn.prepareCall("{call fncget_kmr_inventory(1)}");
			//cstmt.setInt(1, id);
			rs = cstmt.executeQuery();
		} catch (SQLException e) {_log.info("Failed to call function");}
			
		try {
			kStore.binType[id-1] = rs.getByte(1);
			kStore.binQnty[id-1] = rs.getByte(2);
			kStore.boxType[id-1] = rs.getByte(3);
			kStore.boxSN[id-1] = rs.getString("boxsn");
			kStore.boxRlyASN[id-1] = rs.getString("boxrelaya_sn");
			kStore.boxRlyBSN[id-1] = rs.getString("boxrelayb_sn");
			kStore.boxBattSN[id-1] = rs.getString("boxbattery_sn");
			
			//kStore.binType[id-1] kStore.binQnty[id-1] kStore.boxType[id-1] kStore.boxSN[id-1]
			//kStore.boxRlyASN[id-1] kStore.boxRlyBSN[id-1] kStore.boxBattSN[id-1]
			
			_log.info("Returned Storage: " + kStore.binType[id-1] + ", " + kStore.binQnty[id-1] +
					", " + kStore.boxType[id-1] + kStore.boxSN[id-1] + ", " + kStore.boxRlyASN[id-1] +
					", " + kStore.boxRlyBSN[id-1] + ", " + kStore.boxBattSN[id-1]);
			
		} catch (SQLException e) {_log.info("Failed to decode data");}
	}
	
	/*
	private void createReqTable() throws SQLException {
	     String sql = "CREATE TABLE kmr_req " +
	   	   		"(id INT PRIMARY KEY			NOT NULL," +
	   	   		" BinSrcStationNo	INT			NOT NULL," +
	   	   		" BinDestStationNo	INT			NOT NULL," +
	   	   		" BinInfo			CHAR (6)," +
	   	   		" BinLocations		CHAR (14)," +
	   	   		" BinQuantities		CHAR (14)," +
	   	   		" BoxSrcStationNo	INT			NOT NULL," +
   	   			" BoxDestStationNo	INT			NOT NULL," +
   	   			" BoxInfo			CHAR (6)," +
   	   			" BoxLocations		CHAR (14)," +
   	   			" Status 			smallint	NOT NULL," +
   	   			" DTS	timestamp without time zone	NOT NULL)";
	   	stmt.executeUpdate(sql);
	   	_log.info("Table Created\n");
		
	}*/
	/*
	private void createReqTable() throws SQLException {
		stmt.executeUpdate("CREATE TABLE kmr_req " +
	   	"(id INT PRIMARY KEY			NOT NULL," +
	    
	   	" BinSrcStationNo	SMALLINT	NOT NULL," +
	   	" BinDestStationNo	SMALLINT	NOT NULL," +
	   	" BinInfo			CHAR (6)," +
	   	" BinL0				SMALLINT	NOT NULL," +
	   	" BinL1				SMALLINT	NOT NULL," +
	   	" BinL2				SMALLINT	NOT NULL," +
	   	" BinL3				SMALLINT	NOT NULL," +
	   	" BinL4				SMALLINT	NOT NULL," +
	   	" BinL5				SMALLINT	NOT NULL," +
	   	" BinQ0				SMALLINT	NOT NULL," +
	   	" BinQ1				SMALLINT	NOT NULL," +
	   	" BinQ2				SMALLINT	NOT NULL," +
	   	" BinQ3				SMALLINT	NOT NULL," +
	   	" BinQ4				SMALLINT	NOT NULL," +
	   	" BinQ5				SMALLINT	NOT NULL," +
	   	
	   	" BoxSrcStationNo	SMALLINT	NOT NULL," +
   	   	" BoxDestStationNo	SMALLINT	NOT NULL," +
   	   	" BoxInfo			CHAR (6)," +
	   	" BoxL0				SMALLINT	NOT NULL," +
	   	" BoxL1				SMALLINT	NOT NULL," +
	   	" BoxL2				SMALLINT	NOT NULL," +
	   	" BoxL3				SMALLINT	NOT NULL," +
	   	" BoxL4				SMALLINT	NOT NULL," +
	   	" BoxL5				SMALLINT	NOT NULL," +
	   	
   	   	" Status 			smallint	NOT NULL," +
   	   	" DTS	timestamp without time zone	NOT NULL)");
		
	   	_log.info("Table Created\n");
	}*/
	@SuppressWarnings("unused")
	private void createReqTable() throws SQLException {
		stmt.executeUpdate("drop table if exists kmr_req");
		stmt.executeUpdate("CREATE TABLE kmr_req " +
	   	"(id INT PRIMARY KEY			NOT NULL," +
	    
	   	" BinSrcStationNo	SMALLINT	NOT NULL," +
	   	" BinDestStationNo	SMALLINT	NOT NULL," +
	   	" BinInfo			CHAR(6)," +
	   	" BinL				bytea		NOT NULL," +
	   	" BinQ				bytea		NOT NULL," +
	   	
	   	" BoxSrcStationNo	SMALLINT	NOT NULL," +
   	   	" BoxDestStationNo	SMALLINT	NOT NULL," +
   	   	" BoxInfo			CHAR(6)," +
	   	" BoxL				bytea		NOT NULL," +
	   	
   	   	" Status 			smallint	NOT NULL," +
   	   	" DTS	timestamp without time zone	NOT NULL)");
		
	   	_log.info("Table Request Created\n");
	}
	
	@SuppressWarnings("unused")
	private void insertReqDat(int id) throws SQLException {
		
		//prepSt = conn.prepareStatement("INSERT INTO data_table (id, navdata) VALUES (1, ?), (2, ?);");
		prepSt = conn.prepareStatement("INSERT INTO kmr_req VALUES(?, 8, 1, '111000', ?, ?," + 
										" 8, 2, '100000', ?, ?, NOW())");
		
		final byte[] a = {10,20,30,40,50,60}, b = {11,22,33,44,55,66}, c = {1,2,3,4,5,6};
		prepSt.setBytes(2, a);
		prepSt.setBytes(3, b);
		prepSt.setBytes(4, c);
		
		prepSt.setInt(1, id);
		prepSt.setInt(5, id*2);
		prepSt.executeUpdate();
		
		_log.info("Insert to Req Complete");
		
	}
	
	@SuppressWarnings("unused")
	private void updateReqDat(int id, int stat) throws SQLException {
		
		prepSt = conn.prepareStatement("UPDATE kmr_req SET Status = ? WHERE id = ?;");
		
		prepSt.setInt(1, stat);
		prepSt.setInt(2, id);
		
		prepSt.executeUpdate();
		_log.info("Update Request " + id + "Complete");
	}

	@SuppressWarnings("unused")
	private void readReq(int id) throws SQLException {
		
	    prepSt = conn.prepareStatement("SELECT * FROM kmr_req WHERE id = ?");
	    prepSt.setInt(1, id);
	    rs = prepSt.executeQuery();
	    
	    rs.next();
	    binSrc = rs.getByte("BinSrcStationNo");
	    binDest = rs.getByte("BinDestStationNo");
	    binInfo = rs.getString("BinInfo");
	    BinL = rs.getBytes("binl");
	    BinQ = rs.getBytes("BinQ");
	    
	    boxSrc = rs.getByte("BoxSrcStationNo");
	    boxDest = rs.getByte("BoxDestStationNo");
	    boxInfo = rs.getString("BoxInfo");
	    BoxL = rs.getBytes("BoxL");
	    
	    stat = rs.getByte("status");
	}

	@SuppressWarnings("unused")
	private void deleteRow(String tbl, int id) throws SQLException {
		
		if (id == 0) prepSt = conn.prepareStatement("DELETE FROM " + tbl);
		else { 
			prepSt = conn.prepareStatement("DELETE FROM " + tbl + " WHERE id = ?");
			prepSt.setInt(1, id);
		}
		
		//if (prepSt.getParameterMetaData().getParameterCount() > 1) prepSt.setInt(2, id);
		
		int r = prepSt.executeUpdate();
		
		_log.info("row " + id + " deleted from " + tbl + ". " + r + " rows effected"); 
	}
	
	@SuppressWarnings("unused")
	private void clearDat(String tbl) throws SQLException { // incomplete 
		prepSt = conn.prepareStatement("UPDATE ? SET * = 0;");
		prepSt.setString(1, tbl);
		prepSt.executeUpdate();
		_log.info("Clear Complete");
	}
	
	@SuppressWarnings("unused")
	private void printReqDat() {
		
		String dat = "kmr_req table current data\n\n\t";
		dat += binSrc + " -> " + binDest + " - " + binInfo + " - ";
		
		for (byte i = 0; i < numB; ++i) dat += BinL[i] + ",";
		dat += " ";
		for (byte i = 0; i < numB; ++i) dat += BinQ[i] + ",";
		dat += "\n\t" + boxSrc + " -> " + boxDest + " - " + boxInfo + " - "; 
		
		for (byte i = 0; i < numB; ++i) dat += BoxL[i] + ",";
		dat += " - " + stat + "\n\t";
		
		_log.info(dat);
	}
	
	@SuppressWarnings("unused")
	private void postgresConn() {
	     try {
	    	 Class.forName("org.postgresql.Driver");
	    	 
	         conn = DriverManager.getConnection(url, username, pw);
	         
	         if (conn != null) {
	        	 _log.info("Connected to the PostgreSQL server successfully");
	        	 stmt = conn.createStatement();
	         }
	         else { _log.info("Failed to make connection"); }

	     } catch (SQLException e) { }
	     catch (ClassNotFoundException e) { }
	     
	     _log.info("");
	}
	
    ServerSocketData ssD = new ServerSocketData();
	ServerSocketThreadOne st1 = new ServerSocketThreadOne(ssD);
	//Thread t1 = new Thread(st1);

	final int tInt = 4;
	final byte tByte = (byte) tInt;
	
	@Override
	public void dispose() throws Exception { 
		if (conn != null) conn.close();
    	if (stmt != null) stmt.close();
    	if (rs != null) rs.close();
		
    	if (st1 != null) st1.end();
    	
		_log.info("Dispose finished.\n= = = = = = = = = = = = = = = = = = = = = = = = = = = =\n\n");
	}

	@Override
	public void run() throws SQLException {
		_log.info("Run start");
		_log.info("");
		
		postgresConn();
		callKStoreRead(2);
		
		//testTCPServer();
		
		/*
		par.iDat = tInt;
		_log.info("executeRemoteTask start");
		try {
			_rob.lock();
			executeRemoteTask(par);
		} catch (LockException e) {
		} catch (InterruptedException e) {}
		_log.info("");*/
		
		//for (byte i = 0; i < datLen; i++) arrDat[i] = (byte) (i * tByte);
		//tcpConnClient();
		//tcpConnServ();
		//tcpSendMsg();
		//_log.info("Message Received: " + tcpReceiveMsg());
		//sendBytes(arrDat, 0);
		//arrDat = readBytes();
		//_log.info("Received: " + arrDat[0] + ", " + arrDat[1]);
	    
    	
    	/*
    	ReadStartNode();
		ReadTargetNode();
		try {
			//Execute graph motion to station
			_rob.lock();
			executeGraphMo();
			
			//Execute move to location for accurate positioning
			_rob.lock();
			executeMotionToLoc();

		} catch (LockException e) {
			_log.error("Already locked.", e);
		} catch (InterruptedException e) {
			_log.error("Interrupted.", e);
		} finally {
			_rob.unlock();
		}*/
    	
    	/*
		_log.info("");
		_log.info("executeLocalTask start");
		try { executeLocalTask(); }
		catch (InterruptedException e) { _log.error("Interrupted.", e); } 
		*/
		
		
		_log.info("");
		_log.info("Run Complete");
		
  }
	
	private void testTCPServer() {
		st1.start();
		while(st1.isAlive()) {
			
			_log.info("Waiting for a client to connect to the server thread...");
			while(!ssD.isConn());
			_log.info("Connected: " + ssD.getCSocket());
			_log.info("");
		
			byte[] b = null;
			String dt = "";
			_log.info("Logging Messages...");
			while(ssD.isConn()) {
				b = ssD.getData();
				if (b != null) {
					dt = "(" + b.length + ") ";
					for (byte i = 0; i < b.length; i++) dt += b[i] + ", ";
					_log.info(dt);
				}
				ThreadUtil.milliSleep(250);
			}	
			_log.info("Connection Closed: " + !ssD.isConn() + " - " + ssD.getCSocket());
			_log.info("");
		}
		_log.info("Thread Closed: " + !st1.isAlive());
	}

	private void testKMRreq() {
	    //deleteRow("kmr_req", 0);
	    //insertReqDat(1);
	    //insertReqDat(2);
	    //insertReqDat(3);
  
	    /*updateReqDat(1, (tInt+1)<<1);
	    readReq(1);
	    printReqDat();
	    
	    updateReqDat(2, (tInt+2)<<1);
	    readReq(2);
	    printReqDat();
	    
	    updateReqDat(3, (tInt+3)<<1);
	    readReq(3);
	    printReqDat();
	    
	    updateStorDat(2, "boxname", "S070 Box");
	    updateStorDat(1, "boxbit", true);
	    
	    _log.info("Store Returned: " + readKStoreDat(2, "boxname"));*/
	
	    //deleteDat("kmr_req", '1');
	    //deleteDat("kmr_req", '2');
	    //deleteDat("kmr_req", '3');
	}
	
	private void testkStore() {
		postgresConn();
		
	    kStore.binType[0] = tByte << tInt;
	    kStore.binQnty[0] = tByte*3;
	    kStore.boxType[0] = (tByte+1) << tInt;
	    kStore.boxSN[2] = "box sn";
	    
	    kStore.boxRlyASN[2] = "Relay-A 2";
	    kStore.boxRlyBSN[2] = "Relay-B 2";
	    kStore.boxBattSN[2] = "Battery 2";
	    
	    //updateKStorDat(1, "bin");
	    //updateKStorDat(3, "box");
	}
	
	private void testThreadOne() {
		_log.info("before thread: " + kStore.binType[0] + " | " + kStore.binType[1] 
				+ " | " + kStore.binType[2] + " | " + kStore.binType[3]);
		_log.info("");
		
		//MyThreadOne t1 = new MyThreadOne(kStore, tInt);
		new Thread(new MyThreadOne(kStore, tInt)).start();
		
		for (int i = 0; i < tInt; i++) {
			ThreadUtil.milliSleep(1000);
			_log.info("thread after " + (i+1) + " poll: " + kStore.binType[0] + " | " + kStore.binType[1] 
					+ " | " + kStore.binType[2] + " | " + kStore.binType[3]);
			_log.info("");
		}
	}

}