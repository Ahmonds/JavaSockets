package LIFT;

//	KMR On-board Storage
public class KMRstorage {
	
	volatile byte[] binType = {03, 15, 26, 07}, binQnty = {0, 0, 0, 0}, boxType = {0, 0, 0, 0};
	
	volatile String[] boxSN = {"", "", "", ""}, boxRlyASN = {"", "", "", ""}, 
			boxRlyBSN = {"", "", "", ""}, boxBattSN = {"", "", "", ""};

	public void setbinType(int i, byte b) { this.binType[i] = b; }
	public void setbinType(int i, int d) { this.binType[i] = (byte)d; }
	public byte getbinType(int i) { return this.binType[i]; }
}