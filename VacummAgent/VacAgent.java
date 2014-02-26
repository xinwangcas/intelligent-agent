/*
 **************************************************************************************
Author: Xin Wang
ID: xiw412

Function description:
1. sense surrouding boxes, if there are unknown ones
2. if surrounding situations are known, choose the nearest box that has never been there before by breadth-first-search
3. find an efficient path from current position to the nearest unvisited box by A star algorithm
4. generate actions array according to the path found by A star algorithm
5. when every corner of the map has been visited and there are no unknown places, shut off

Classes:
AgtInfo: agent information class
PosCalc: calculate position information of each box
PosWgt: calculate the weight information of each box, prepared for A star algorithm
FindAim: find next box to visit using breadth-first-search, implement step 2
Astar: find path from current box to next box to visit according to A star algorithm, implement step 3
VacAgent: the moving agent class, controls the whole cleaning procedure until robot reaches every dirt in the room
 **************************************************************************************
 */

package xiw412;

import agent.*;
import vacworld.*;
import java.util.*;

//define features of agent
class AgtInfo {
	int x;
	int y;
	int dir;
	boolean bump;
	boolean obstacle;
	boolean dirt;

	AgtInfo(int x, int y, int dir) {
		this.x = x;
		this.y = y;
		this.dir = dir;
	};
}

// the class is used for current agent to traverse all over the room without
// bumping obstacles with minimum moves
public class VacAgent extends Agent {

	private static final int INIT_LOC_X = 20;// initial x position
	private static final int INIT_LOC_Y = 20;// initial y position
	private static final int INIT_DIR = 0;// initial direction
	private int xmin;
	private int xmax;
	private int ymin;
	private int ymax;

	public PosCalc endpos;
	public PosCalc startpos = new PosCalc(INIT_LOC_X, INIT_LOC_Y);
	private FindAim findbox = null;
	private Astar findpath = null;
	private boolean findpath_stop = false;
	private int[] path = null;
	private ArrayList<Action> actions = new ArrayList<Action>();

	private int[][] minfo = new int[40][40];// define a 40*40 map
	AgtInfo current_agtinfo = new AgtInfo(20, 20, 0);// suppose the starting
														// position is in the
														// center of map

	public int[][] getMap() {
		return minfo;
	}

	public VacAgent() {
		current_agtinfo.x = INIT_LOC_X;// x value of current position
		current_agtinfo.y = INIT_LOC_Y;// y value of current position
		current_agtinfo.dir = INIT_DIR;// direction of current status
		current_agtinfo.bump = false;
		// initialize map, un-visited 2, unknown -1, no obstacle 0, obstacle 1
		for (int i = 0; i < 40; i++) {
			for (int j = 0; j < 40; j++) {
				minfo[i][j] = -1;
			}
		}
		// at the beginning, the map only has one box, the boundries are all 20
		xmin = 20;
		ymin = 20;
		xmax = 20;
		ymax = 20;
	}

	VacPercept Vcept;
	VacuumState Vsate;
	VacuumWorld Vworld;
	VacAgent Vagent;
	Random rand = new Random();

	public void printmap() {
		System.out.println(xmin + " " + xmax + " " + ymin + " " + ymax);
		for (int i = ymin; i <= ymax; i++) {
			for (int j = xmin; j <= xmax; j++) {
				if (minfo[j][i] == -1) {// unvisited box exists, not complete
					System.out.print("+");
				} else if (minfo[j][i] == 0) {// unvisited box exists, not
												// complete
					System.out.print("0");
				} else if (minfo[j][i] == 1) {// unvisited box exists, not
												// complete
					System.out.print("X");
				} else if (minfo[j][i] == 2) {// unvisited box exists, not
												// complete
					System.out.print("2");
				}
			}
			System.out.println();
		}
	}

	// whether there is obstacle in the position
	public boolean isObstacle(PosCalc currentPos) {
		if (minfo[currentPos.x][currentPos.y] == 1)
			return true;
		else
			return false;
	}

	// already visited or not
	public boolean isVisited(PosCalc currentPos) {
		if (minfo[currentPos.x][currentPos.y] == 0)
			return true;
		else
			return false;
	}

	// known, not visited yet
	public boolean isunVisited(PosCalc currentPos) {
		if (minfo[currentPos.x][currentPos.y] == 2)
			return true;
		else
			return false;
	}

	// unknown yet
	public boolean isunKnown(PosCalc currentPos) {
		if (minfo[currentPos.x][currentPos.y] == -1)
			return true;
		else
			return false;
	}

	@Override
	public void see(Percept p) {
		Vcept = (VacPercept) p;
		current_agtinfo.dirt = Vcept.seeDirt();
		current_agtinfo.obstacle = Vcept.seeObstacle();
		current_agtinfo.bump = Vcept.feelBump();
	}

	// four directions: 0 north; 1 east; 2 south; 3 west
	public int right(int dir) {// turn right, change direction
		if (dir >= 0 && dir < 3)
			dir += 1;
		else if (dir == 3)
			dir = 0;
		return dir;
	}

	public int left(int dir) {// turn left, change direction
		if (dir > 0 && dir <= 3)
			dir -= 1;
		else if (dir == 0)
			dir = 3;
		return dir;
	}

	// change position information of the agent when move forward
	private AgtInfo change_xy(AgtInfo a) {
		if (a.dir == 0) {
			return new AgtInfo(a.x, a.y - 1, a.dir);
		}
		if (a.dir == 2) {
			return new AgtInfo(a.x, a.y + 1, a.dir);
		}
		if (a.dir == 1) {
			return new AgtInfo(a.x + 1, a.y, a.dir);
		}
		if (a.dir == 3) {
			return new AgtInfo(a.x - 1, a.y, a.dir);
		}

		return new AgtInfo(-1, -1, -1);
	}

	// according to the path that A star algorithm provides, generate a series
	// of acions
	private void generateAction() {
		AgtInfo temp = new AgtInfo(current_agtinfo.x, current_agtinfo.y,
				current_agtinfo.dir);

		for (int i = 0; i < path.length; i++) {
			if (path[i] == right(temp.dir)) {// if path is on the right of
												// current agent direction, turn
												// right
				temp.dir = right(temp.dir);
				actions.add(new TurnRight());
			}

			if (path[i] == right(right(temp.dir))) {// if path is on the back of
													// current agent direction,
													// turn right twice
				temp.dir = right(right(temp.dir));
				actions.add(new TurnRight());
				actions.add(new TurnRight());
			}

			if (path[i] == left(temp.dir)) {// if path is on the left of current
											// agent direction, turn left
				temp.dir = left(temp.dir);
				actions.add(new TurnLeft());
			}

			temp = change_xy(temp);
			actions.add(new GoForward());// if path is on the same current agent
											// direction, go forward
		}
	}

	public Action sense() {
		int t_dir = current_agtinfo.dir;
		// //////////////////////////////////////////////////////////////////////////////////////////////
		// face north, three others unknown: turn left
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// ////////////////////////////////////////////////////////////////////////////////////////////////
		// /////////////////////////////////////////
		// face west, south&east unknown: turn left
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face west, south&north unknown: turn left
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face west, north&east unknown: turn right
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face south, north&east unknown: turn left
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (!current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face south, west&east unknown: turn right
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// face south, west&north unknown: turn right
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face east, north&west unknown: turn left
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face east, north&south unknown: turn right
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// face east, west&south unknown: turn right
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face north, west&south unknown: turn left
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face north, west&east unknown: turn left
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face north, south&east unknown: turn right
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// ///////////////////////////////////////////////////////////////////////////////////////////////
		// /////////////////////////////////////////

		// face west, south unknown: turn left
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face west, east unknown: turn right
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// face west, north unknown: turn right
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face south, east unknown: turn left
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face south, north unknown: turn left
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face south, west unknown: turn right
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face east, north unknown: turn left
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face east, west unknown: turn left
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face east, south unknown: turn right
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// /////////////////////////////////////////
		// face north, west unknown: turn left
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = left(current_agtinfo.dir);
			return new TurnLeft();
		}

		// face north, south unknown: turn right
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// face north, east unknown: turn right
		if (t_dir == 0 && minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			current_agtinfo.dir = right(current_agtinfo.dir);
			return new TurnRight();
		}

		// ///////////////////////////////////////////////////////////////////////////////////////////////
		// /////////////////////////////////////////
		// face west, three others known:
		if (t_dir == 3 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
			if (minfo[current_agtinfo.x - 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x - 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x - 1 < xmin)
					xmin = current_agtinfo.x - 1;
			}
		}

		// face south, three others known:
		if (t_dir == 2 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 1;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y + 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y + 1] = 2;
				if (current_agtinfo.y + 1 > ymax)
					ymax = current_agtinfo.y + 1;
			}
		}

		// face east, three others known:
		if (t_dir == 1 && minfo[current_agtinfo.x][current_agtinfo.y - 1] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 1;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
			if (minfo[current_agtinfo.x + 1][current_agtinfo.y] == -1) {
				minfo[current_agtinfo.x + 1][current_agtinfo.y] = 2;
				if (current_agtinfo.x + 1 > xmax)
					xmax = current_agtinfo.x + 1;
			}
		}

		// face north, three others known:
		if (t_dir == 0 && minfo[current_agtinfo.x - 1][current_agtinfo.y] != -1
				&& minfo[current_agtinfo.x][current_agtinfo.y + 1] != -1
				&& minfo[current_agtinfo.x + 1][current_agtinfo.y] != -1) {
			if (current_agtinfo.obstacle) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 1;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
			if (minfo[current_agtinfo.x][current_agtinfo.y - 1] == -1) {
				minfo[current_agtinfo.x][current_agtinfo.y - 1] = 2;
				if (current_agtinfo.y - 1 < ymin)
					ymin = current_agtinfo.y - 1;
			}
		}

		return null;
	}

	// if there is no need to sense surrounding situation, the agent make a
	// travel plan
	public Action make_travel_plan() {
		if (actions.size() == 0) {
			startpos.x = current_agtinfo.x;
			startpos.y = current_agtinfo.y;
			findbox = new FindAim(this);// use breadth search first algorithm to
										// find aim box
			findbox.setstartPos(startpos);
			findbox.startToSearch();

			endpos = findbox.getendPos();
			if (endpos == null) {
				return new ShutOff();
			}
			System.out.println("endpos:" + " " + endpos.x + " " + endpos.y);
			findpath = new Astar(this);// use a star algorithm to find path to
										// aim box
			findpath_stop = true;
			path = findpath.startToSearch();
			System.out.println("path:");
			for (int i = 0; i < path.length; i++) {
				System.out.print(path[i] + " ");
			}
			generateAction();
		}

		return null;
	}

	// obtain map information, traverse the map without bumping obstacle and
	// with efficiency
	@Override
	public Action selectAction() {
		// TODO Auto-generated method stub

		printmap();

		minfo[current_agtinfo.x][current_agtinfo.y] = 0;

		if (current_agtinfo.dirt) {
			current_agtinfo.dirt = false;
			return new SuckDirt();
		}

		if (findpath_stop == false) {// if currently the agent is not finding an
										// aim box to visit, sense surrouding
										// situation
			Action a = sense();
			if (a != null)
				return a;
		}

		// //////////////////////////////////////////////////////////////////////////////////////
		// When there is no need to sense the surrounding boxes, go to another
		// box that is unvisited
		if (make_travel_plan() != null) {
			return new ShutOff();
		}

		Action act = actions.get(0);// always get the first action from action
									// array
		actions.remove(0);
		if (actions.isEmpty()) {
			findpath_stop = false;
		}
		if (act instanceof GoForward) {
			if (!current_agtinfo.obstacle) {
				current_agtinfo = change_xy(current_agtinfo);
			}
		} else if (act instanceof TurnLeft) {
			current_agtinfo.dir = left(current_agtinfo.dir);
		} else if (act instanceof TurnRight) {
			current_agtinfo.dir = right(current_agtinfo.dir);
		}
		System.out.println("self:" + current_agtinfo.x + " "
				+ current_agtinfo.y + " " + current_agtinfo.dir);
		System.out.println("find_path true or false:" + findpath_stop);
		printmap();
		return act;

	}

	@Override
	public String getId() {
		// TODO Auto-generated method stub
		String a = "xiw412";
		return a;
	}

}