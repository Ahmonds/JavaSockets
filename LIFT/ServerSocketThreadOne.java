package LIFT;

import java.net.*;
//import java.util.Timer;
//import java.util.TimerTask;
import java.io.*;

public class ServerSocketThreadOne extends Thread{

	private boolean exit = false, er = false;
	private ServerSocketData ssD = null;
	
	private Socket cSocket = null;
	private ServerSocket sSocket = null;
	private OutputStream output = null;
	private InputStream input = null;
	//private Timer timer = new Timer();
	
	public ServerSocketThreadOne(ServerSocketData d) { this.ssD = d; }
	
	private boolean tcpConnServer() {
		try {
	    	cSocket = sSocket.accept();
	    	ssD.setSSocket(sSocket.toString());
	    	ssD.setCSocket(cSocket.toString());
	    	 
	    	input = cSocket.getInputStream();
	    	output = cSocket.getOutputStream();
	    	ssD.reportConn(true);
	    	exit = false;
	    	er = false;
	    	ssD.setStat("Server connected");
	    	 
		} catch (SocketTimeoutException e) {ssD.setStat("Thread connection timed out"); er = true; return false; }
		catch (IOException e) {ssD.setStat("Thread failed connection"); er = true; return false; }
		return true;
	}

	public boolean sendBytes(byte[] bArr) {
		this.interrupt();
	    if (bArr.length == -1) return false;
	    
	    try {
		    output.write(bArr);
		    
	    } catch (IOException e) { return false; }
	    return true;
	}
	private boolean readBytes() {
		try {
			//ssD.reportRdy(false);
			int len = input.read();
			byte[] bArr = null;
			
			if (len > 0) bArr = new byte[len];
			else if(len == 0) { exit = true; return false; }
			else return false;
			
		    input.read(bArr, 0, len);
		    ssD.setData(bArr);
		    ssD.reportRdy(true);

		} catch (IOException e) { return false; }
		return true;
	}

	
	public void run() {
		try {
			sSocket = new ServerSocket(ssD.getPort());
			sSocket.setSoTimeout(10000);
		} catch (IOException e) {}
		
		while(!exit) {
			exit = true;
			ssD.setStat("Server waiting for connection");
			if (tcpConnServer()) {
				do { exit = Thread.interrupted(); //exit = Thread.currentThread().isInterrupted();
				} while(!exit && readBytes());

				if (cSocket != null) {
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
		}
		
		try {
			sSocket.close();
			if (!er) ssD.setStat("Thread closed normally");
		} catch (IOException e) {}
		return;
	}
}