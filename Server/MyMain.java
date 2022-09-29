
import java.net.*;
import java.io.*;

import java.util.Collection;
import java.util.Timer;
import java.util.TimerTask;
import javax.inject.Inject;
 
public class MyMain {

	@Inject
	private ITaskLogger _log;
	
    	ServerSocketData ssD = new ServerSocketData();
	ServerSocketThreadOne st1 = new ServerSocketThreadOne(ssD);

	final int tInt = 3;
	final byte tByte = 4;
	
	public static void Main() {
		_log.info("Run start");
		_log.info("");
		
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
}
