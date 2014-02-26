package xiw412;

import pacworld.VisibleAgent;
/**********************************
 * 
 * @author xinwang
 *
 *agent table that each agent remember
 **********************************/
public class tAgents{
	public String tid;
	public int x;
	public int y;
	public int dest_x;
	public int dest_y;
	public int HoldPac;
	public int PacID;
	
	public tAgents(String id){
		this.tid = id;
	}
	public tAgents(String id, int loc_x, int loc_y){
		this.tid = id;
		this.x = loc_x;
		this.y = loc_y;
	}
	
	public tAgents(VisibleAgent v){
		this.tid = v.getId();
		this.x = v.getX();
		this.y = v.getY();
	}
}