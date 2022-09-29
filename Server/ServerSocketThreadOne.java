package LIFT;

import java.net.*;
//import java.util.Timer;
//import java.util.TimerTask;
import java.io.*;

public class ServerSocketThreadOne extends Thread{

	private boolean exit = false, er = false; 	// 'er' used to not override status when thread terminates
	private ServerSocketData ssD = null;
	
	private Socket cSocket = null;
	private ServerSocket sSocket = null;
	private OutputStream output = null;
	private InputStream input = null;
	//private Timer timer = new Timer();
	
	public ServerSocketThreadOne(ServerSocketData d) { this.ssD = d; }
	
	private boolean tcpConnServer() {	// Server-side connection method 
		try {
			sSocket.setSoTimeout(15*1000); 			// Timer throws timeout exception after 5*1000ms
	    	try { cSocket = sSocket.accept(); 			// Block thread waiting for connection
	    	} catch (SocketTimeoutException e) {
	    		ssD.setStat("Thread connection timed out"); 
	    		er = true; 
	    		return false;
	    	}
	    	ssD.setSSocket(sSocket.toString());
	    	ssD.setCSocket(cSocket.toString());
	    	 
	    	input = cSocket.getInputStream();
	    	output = cSocket.getOutputStream();
	    	ssD.reportConn(true);
	    	exit = false;
	    	er = false;
	    	ssD.setStat("Server connected");
	    	 
		} catch (IOException e) {ssD.setStat("Thread failed connection"); er = true; return false; }
		return true;
	}

	public boolean sendBytes(byte[] bArr) {
	    if (bArr.length == -1) return false;
	    try { output.write(bArr); 
	    } catch (IOException e) { return false; }
	    catch (NullPointerException e) { return false; }
	    return true;
	}
	private boolean readBytes() {	// Reads from current connected client socket 
		try {
			int len = 0;
			cSocket.setSoTimeout(100);		// Timeout read() to unblock
		    try { len = input.read();
		    } catch (SocketTimeoutException e) {}
			
			byte[] bArr = null;
			
			if (len > 1) bArr = new byte[len];
			else if(len == 0) return true;
			else if (len == 1) {exit = true; return false;}	// My exit signal to terminate server
			else return false;	// if == -1, then input stream is disconnected
			
		    input.read(bArr, 0, len);		// Read the rest if the frame
		    ssD.setData(bArr);				// Update shared memory
		    ssD.reportRdy(true);

		} catch (IOException e) { return false; }
		return true;
	}

	
	public void run() {
		try { sSocket = new ServerSocket(ssD.getPort());	// Create server socket once
		} catch (IOException e) {}
		
		while(!exit) {			// Server running
			exit = true;
			ssD.setStat("Server waiting for connection");
			
			if (tcpConnServer()) {	
				do { exit = Thread.interrupted();	// Check and clear interrupt flag
				} while(!exit && readBytes());		// Run while there is a client connected

				if (cSocket != null) {	// Close socket after disconnect
					try {
						input.close();
						output.close();
						cSocket.close();
					} catch (IOException e) {}
				}
				ssD.reportConn(false);
				ssD.setCSocket(null);
				ssD.setData(null);
			}
		}						// Server running 
		
		try {
			sSocket.close();	// Close server
			if (!er) ssD.setStat("Thread closed normally");
		} catch (IOException e) {}
		
		return;	// Terminate thread
	}
}