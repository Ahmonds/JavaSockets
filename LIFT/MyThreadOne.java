package LIFT;


public class MyThreadOne implements Runnable {
	
	private int t = 0;
	private KMRstorage obj = null;
	
	public MyThreadOne(KMRstorage o, int t) {
		this.obj = o;
		this.t = t;
	}
	
	
	public void run() {
		
		int tt = 5000/t;
		
		for(int i = 0; i < t; i++) {
		
			try {
				obj.setbinType(i, (i+t)*10);
				Thread.sleep(tt);
			}
        	catch (Exception e) {}
		}
		return;
	}
	
	
}
