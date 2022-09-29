package LIFT;

import java.net.*;
import java.io.*;

public class ClientSocketThreadOne implements Runnable{

	private boolean exit;
	private ClientSocketData csD = null;
	private KMRstorage kStore = null;
	
	Socket cSocket = null;
	ServerSocket sSocket = null;
	OutputStream output = null;
	InputStream input = null;
	
	DataOutputStream dos = null;
	DataInputStream dis = null;
	PrintWriter writer = null;
	BufferedReader reader = null;
	
	public ClientSocketThreadOne(ClientSocketData d, KMRstorage k) {
		this.kStore = k;
		this.csD = d;
	}
	public ClientSocketThreadOne(ClientSocketData d) { this.csD = d; }
	
	private void tcpConnClient() {
	     try { 
	    	 cSocket = new Socket(csD.getIP(), csD.getPort());
	    	 if (cSocket != null) { 
	    		 csD.reportConn(cSocket.isConnected());
		    	 csD.setCSocket(cSocket.toString());
	    	 }
	     } catch (IOException e) { }
	}

	public boolean sendBytes(byte[] bArr) {
	    if (bArr.length == -1) return false;
	    
	    try {
		    output.write(bArr);
		    
	    } catch (IOException e) { return false; }
	    return true;
	}

	public boolean readBytes() {
		try {
			int len = input.read();
			byte[] bArr = null;
			
			if (len > 0) bArr = new byte[len];
			else if(len == 0) return false;
			else { 
				input.close();
				output.close();
				cSocket.close();
				csD.reportConn(false);
				exit = true;
				return false;
			}
			
		    input.read(bArr, 0, len);
		    csD.setData(bArr);

		} catch (IOException e) { return false; }
		return true;
	}
	
	private void tcpSendMsg() {
   	 	try {
   	 		output = cSocket.getOutputStream();
   	   	 	writer = new PrintWriter(output, true);
   			writer.println(csD.getMsg());
   	 	}
   	 	catch (IOException e) { }
	}
	private String tcpReceiveMsg() {
		String line = "";
		try {
			input = cSocket.getInputStream();
	    	reader = new BufferedReader(new InputStreamReader(input));
	    	line = reader.readLine();
	    	if (line == null) { cSocket.close(); exit = true; }
		} catch (IOException e) { }
		
		return line;
	}
	
	public void run() {
		exit = false;
		
		tcpConnClient();
		
		while(!exit) readBytes();
		
		csD.setMsg("Thread Closed");
		csD.setCSocket(null);
		return;
	}
	
	public void stop() { exit = true; }
	
}
