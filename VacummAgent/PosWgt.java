package xiw412;

public class PosWgt {// define the relationship among current, start node, end
						// node and weights on the path in A*

	public PosCalc cur_pos;// current box

	// distance from start box to current box;
	private int start_dist;

	// distance from current box to end box;
	private int end_dist;

	// weight of father box, which is the last step box of current box
	private PosWgt father;

	// total cost f=g+h;
	private int cost;

	public PosWgt(PosCalc pos) {
		this.cur_pos = pos;
	}

	// get the distance information from start and end boxes, update father box
	public PosWgt(PosCalc pos, PosWgt father, PosWgt start, PosWgt end) {
		this(pos);
		dist_dest(end);
		dist_start(start);
		setFather(father);
		setCost(getstart_dist() + getend_dist());
	}

	// calculate distance from last step father box to current box
	public int get_dist_father(PosWgt father) {
		PosCalc fatherPos = father.getPosCalc();
		return fatherPos.dist(getPosCalc());
	}

	// set the newly input box as father box, update the distance to start node
	// as from father to start box
	// plus ditance from current box to father
	public void update_father(PosWgt father) {
		setFather(father);
		int dist = get_dist_father(father);
		setstart_dist(dist + father.getstart_dist());
		setCost(getstart_dist() + getend_dist());
	}

	public PosCalc getPosCalc() {
		return cur_pos;
	}

	public PosWgt getFather() {
		return father;
	}

	public int getCost() {
		return cost;
	}

	public int getstart_dist() {
		return start_dist;
	}

	// set distance to destination
	void dist_start(PosWgt start) {
		PosCalc targetc = start.getPosCalc();
		int distanceToTarget = getPosCalc().dist(targetc);
		setstart_dist(distanceToTarget);
	}

	// set distance to destination
	void dist_dest(PosWgt target) {
		PosCalc targetc = target.getPosCalc();
		int distanceToTarget = getPosCalc().dist(targetc);
		setend_dist(distanceToTarget);
	}

	void setstart_dist(int start_dist) {
		this.start_dist = start_dist;
	}

	int getend_dist() {
		return end_dist;
	}

	private void setend_dist(int end_dist) {
		this.end_dist = end_dist;
	}

	private void setFather(PosWgt father) {
		this.father = father;
	}

	void setCost(int cost) {
		this.cost = cost;
	}

	public int getX() {
		return cur_pos.x;
	}

	public int getY() {
		return cur_pos.y;
	}

	public int dir_calc(PosWgt pos) // calculate direction info of the node
	{
		int dir = -1;
		int verticalDistance = getY() - pos.getY();
		int horizontalDistance = getX() - pos.getX();
		if (verticalDistance == 0 && horizontalDistance == 1)
			dir = 3;
		if (verticalDistance == 0 && horizontalDistance == -1)
			dir = 1;
		if (verticalDistance == 1 && horizontalDistance == 0)
			dir = 0;
		if (verticalDistance == -1 && horizontalDistance == 0)
			dir = 2;
		return dir;
	}

}