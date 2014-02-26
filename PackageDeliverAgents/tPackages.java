package xiw412;

import pacworld.VisiblePackage;

/*****************************************
 * author: Xin Wang
 * *package table that each agent remember
 *****************************************/
public class tPackages{
	public int pid;
	public int x;
	public int y;
	public int dest_x;
	public int dest_y;
	public boolean isHeld;
	public boolean isOrdered;//is reserved by an agent or not
	public boolean isPicked;//is picked up or not
	public boolean isDropped;//is dropped off or not
	
	public tPackages(int id, int loc_x, int loc_y){
		this.pid = id;
		this.x = loc_x;
		this.y = loc_y;
	}
	
	public tPackages(VisiblePackage v){
		this.pid = v.getId();
		this.x = v.getX();
		this.y = v.getY();
		this.dest_x = v.getDestX();
		this.dest_y = v.getDestY();
		this.isHeld = v.isHeld();
	}
	
	   public String toString() {
		      return "Package " + pid + ": Loc=(" + x + ", " + y + "), " +
		      "Dest=(" + dest_x + ", " + dest_y + "), Held=" + isHeld;
		   }
}