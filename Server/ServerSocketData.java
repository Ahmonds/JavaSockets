package LIFT;

public class ServerSocketData {	// Shared data for server socket thread
	
	private volatile String sStr = null;
	public void setSSocket(String a) { this.sStr = a; }
	public String getSSocket() { return this.sStr; }
	
	private volatile String cStr = null;
	public void setCSocket(String b) { this.cStr = b; }
	public String getCSocket() { return this.cStr; }
	
	private int pt = 2100;
	public void setPort(int c) { this.pt = c; }
	public int getPort() { return this.pt; }
	
	private String ip = "192.168.135.129";
	public void setIP(String d) { this.ip = d; }
	public String getIP() { return this.ip; }

	private volatile boolean con = false;
	public void reportConn(boolean e) { this.con = e; }
	public boolean isConn() { return this.con; }
	
	private volatile byte[] data = null;
	public void setData(byte[] f) { this.data = f; }
	public byte[] getData() { return this.data; }
	
	private volatile String stat = "";
	public void setStat(String g) { this.stat = g; }
	public String getStat() { return this.stat; }
	
	private volatile boolean rdy = false;
	public void reportRdy(boolean h) { this.rdy = h; }
	public boolean isRdy() { return this.rdy; }
}
