package LIFT;

import java.net.*;
import java.io.*;
import java.sql.*;

import java.util.Collection;
import java.util.Iterator;
import java.util.List;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

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
	private PreparedStatement prepSt = null;
	private CallableStatement cstmt = null;
	private ResultSet rs = null;
	final String url = "jdbc:postgresql://localhost:5432/kmrdb";
	final String username = "postgres", pw = "kuka"; 
	
    //	TCP Request Data
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
		private int iDat = 0;
		private String sDat = "Test";
		
		private void writeObject(java.io.ObjectOutputStream out) throws IOException {}
		private void readObject(java.io.ObjectInputStream in) throws IOException, ClassNotFoundException {}
		private void readObjectNoData() throws ObjectStreamException {}
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
		/*ObjectOutputStream out = null;
		try {
			FileOutputStream fos = new FileOutputStream("/tmp/iiwaPar.ser");
			out = new ObjectOutputStream(fos);
			out.writeObject(p);
		} catch (IOException e) {}*/
		
		final String s = "application.MoveToDrivePos";
		RemoteTaskId remoTaskId = new RemoteTaskId(s);
		TaskRequest request = new TaskRequest(remoTaskId, p);
		//TaskRequest request = new TaskRequest(remoTaskId);
		
		TaskRequestContainer container = _rob.execute(request);
		container.awaitFinished();
		
		_log.info("Rmote task called");
	}
	private void executeRemoteTask() {
		final String taskString = "application.MoveToDrivePos";
		
		RemoteTaskId remoTaskId = new RemoteTaskId(taskString);
		TaskRequest request = new TaskRequest(remoTaskId);
		
		TaskRequestContainer container = _rob.execute(request);
		container.awaitFinished();
		
		_log.info("Rmote task called");
	}

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
	
	
	@SuppressWarnings("unused")
	private void callKStoreUpdate(int id, String bb) {
		
		try {
			cstmt = conn.prepareCall("{call procupdatekmrinventory" +
									"(?, ?, ?, ?, ?,  ?, ?, ?, ?)}");
						//(pid integer, pflag text, pbintype smallint, pbinqty smallint, pboxtype smallint,
									// pboxsn text, pboxasn text, pboxbsn text, pboxbatsn text)
			cstmt.setInt(1, id);
			cstmt.setString(2, bb);
			cstmt.setByte(3, kStore.binType[id-1]);
			cstmt.setByte(4, kStore.binQnty[id-1]);
			cstmt.setByte(5, kStore.boxType[id-1]);
			cstmt.setString(6, kStore.boxSN[id-1]);
			cstmt.setString(7, kStore.boxRlyASN[id-1]);
			cstmt.setString(8, kStore.boxRlyBSN[id-1]);
			cstmt.setString(9, kStore.boxBattSN[id-1]);
			
			cstmt.executeUpdate();
			_log.info("Update Row " + id + " Complete");
			
		} catch (SQLException e) {_log.info("Failed to call update function");}
	}

	private void callKStoreRead(int id) {
		try {
			//prepSt = conn.prepareStatement("SELECT fncget_kmr_inventory(id => ?)");
			cstmt = conn.prepareCall("{call fncget_kmr_inventory(?)}");
			cstmt.setInt(1, id);
			rs = cstmt.executeQuery();
		} catch (SQLException e) {_log.info("Failed to call read function");}
			
		try {
			rs.next();
			kStore.binType[id-1] = rs.getByte(1);
			kStore.binQnty[id-1] = rs.getByte(2);
			kStore.boxType[id-1] = rs.getByte(3);
			kStore.boxSN[id-1] = rs.getString(4);
			kStore.boxRlyASN[id-1] = rs.getString(5);
			kStore.boxRlyBSN[id-1] = rs.getString(6);
			kStore.boxBattSN[id-1] = rs.getString(7);
			
			//kStore.binType[id-1] kStore.binQnty[id-1] kStore.boxType[id-1] kStore.boxSN[id-1]
			//kStore.boxRlyASN[id-1] kStore.boxRlyBSN[id-1] kStore.boxBattSN[id-1]
			
			_log.info("Returned Storage [" + id + "]: " + kStore.binType[id-1] + ", " + kStore.binQnty[id-1] +
					", " + kStore.boxType[id-1] + ", " + kStore.boxSN[id-1] + ", " + kStore.boxRlyASN[id-1] +
					", " + kStore.boxRlyBSN[id-1] + ", " + kStore.boxBattSN[id-1]);
			
		} catch (SQLException e) {_log.info("Failed to decode data");}
	}
	
	@SuppressWarnings("unused")
	private void postgresConn() {
	     try {
	    	 Class.forName("org.postgresql.Driver");
	    	 
	         conn = DriverManager.getConnection(url, username, pw);
	         
	         if (conn != null) _log.info("Connected to the PostgreSQL server successfully");
	         else _log.info("Failed to make connection");

	     } catch (SQLException e) { }
	     catch (ClassNotFoundException e) { }
	     
	     _log.info("");
	}
	
	
    ServerSocketData ssD = new ServerSocketData();
	ServerSocketThreadOne st1 = new ServerSocketThreadOne(ssD);

	final int tInt = 3;
	final byte tByte = 4;
	
	@Override
	public void dispose() throws Exception {
		if (conn != null) conn.close();
    	if (rs != null) rs.close();
    	
		_log.info("Dispose finished.\n= = = = = = = = = = = = = = = = = = = = = = = = = = = =\n\n");
	}

	@Override
	public void run() throws SQLException {
		_log.info("Run start");
		_log.info("");
		
		//testkStore();
		
		Timer timer = new Timer();
		timer.schedule(new TimerTask() {
			public void run() {
				byte[] bb = {tByte, tByte << 2, (tByte+1)*3, tByte << 4, 0, tByte+5*tByte, 127};
				if (st1.sendBytes(bb)) _log.info(" - Write Interrupted - ");
			}
		}, 8*1000, 3*1000);

		//timer.cancel();
		testTCPServer(200);
		timer.cancel();
		
		/*
		par.iDat = tInt;
		_log.info("executeRemoteTask start");
		try {
			_rob.lock();
			executeRemoteTask(par);
		} catch (LockException e) {
		} catch (InterruptedException e) {}
		_log.info("");*/
    	
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
	
	private void testTCPServer(int t) {
		st1.start();
		while(st1.isAlive()) {
			
			_log.info("Waiting for a client to connect to the server thread...");
			while(!ssD.isConn() && st1.isAlive());
			_log.info("Connected to " + ssD.getCSocket());
			_log.info("");

			byte[] b = null;
			String dt = "";
			_log.info("Logging Messages...");
			while(ssD.isConn()) {
				if (ssD.isRdy()) {
					b = ssD.getData();
					dt = "(" + b.length + ") ";
					for (byte i = 0; i < b.length; i++) dt += b[i] + ", ";
					_log.info(dt);
					ssD.reportRdy(false);
				}
				ThreadUtil.milliSleep(t);
			}
			_log.info("Connection Closed: " + !ssD.isConn() + " - " + ssD.getCSocket());
			_log.info("");
		}
		_log.info(ssD.getStat());
	}
	
	private void testkStore() {
	    postgresConn();
		for (int i = 1; i <= 4; i++) callKStoreRead(i);
		_log.info("");
		
		kStore.binType[1] = tByte << tInt;
	    kStore.binQnty[1] = (tByte+tInt)*3;
	    kStore.boxType[1] = (tByte+1) << tInt-1;
	    kStore.boxSN[1] = "BoxSN"+tInt;
	    
	    kStore.boxRlyASN[1] = "Relay-A"+tInt;
	    kStore.boxRlyBSN[1] = "Relay-B"+tInt;
	    kStore.boxBattSN[1] = "Battery-"+tInt;
	    
	    kStore.binType[3] = (tByte-1) << tInt;
	    kStore.binQnty[3] = tByte+tInt;
	    kStore.boxType[3] = (tByte) << tInt;
		
		callKStoreUpdate(2, "both");
		callKStoreUpdate(4, "bin");
		for (int i = 1; i <= 4; i++) callKStoreRead(i);
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