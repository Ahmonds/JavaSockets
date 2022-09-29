package LIFT;

public class ClientSocketData {
	
	private volatile String cStr = null;
	public void setCSocket(String a) { this.cStr = a; }
	public String getCSocket() { return this.cStr; }
	
	private volatile String msg = "";
	public void setMsg(String b) { this.msg = b; }
	public String getMsg() { return this.msg; }
	
	private volatile int pt = 2000;
	public void setPort(int c) { this.pt = c; }
	public int getPort() { return this.pt; }
	
	private volatile String ip = "192.168.135.128";
	public void setIP(String d) { this.ip = d; }
	public String getIP() { return this.ip; }
	
	private volatile boolean con = false;
	public void reportConn(boolean e) { this.con = e; }
	public boolean isConn() { return this.con; }
	
	private volatile byte[] data;
	public void setData(byte[] f) { this.data = f; }
	public byte[] getData() { return this.data; }
}
