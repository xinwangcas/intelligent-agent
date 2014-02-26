package xiw412;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.util.Stack;

//This class implements the A star algorithm in finding path 
public class Astar {

	private VacAgent agt;
	// unvisted boxes that are to be visited soon
	private List<PosWgt> openList;

	// visited boxes are in close list
	private List<PosWgt> closeList;

	// posistion of the unvisited box the agent plan to go next
	private PosWgt endPosition;

	// position of the box the agent starts in the round
	private PosWgt startPosition;

	// path arrays from current box to aim box
	private int[] path;

	// temperary stack to trace back path from aim box to original box according
	// to A star algorithm
	private Stack<Integer> pa;

	private PosWgt goal_inOpen;

	// offset of one move
	private List<PosCalc> offsets = Arrays.asList(new PosCalc[] {
			new PosCalc(1, 0), new PosCalc(0, 1), new PosCalc(-1, 0),
			new PosCalc(0, -1) });

	// constructor of Astar
	public Astar(VacAgent vac) {
		this.agt = vac;
		this.openList = new ArrayList<PosWgt>();
		this.closeList = new ArrayList<PosWgt>();
		agt.getMap();
		PosWgt start = new PosWgt(vac.startpos);
		PosWgt end = new PosWgt(vac.endpos);
		pa = new Stack<Integer>();
		this.startPosition = start;
		this.endPosition = end;
		this.startPosition.setstart_dist(0);
		this.startPosition.dist_dest(end);
		this.startPosition.setCost(this.startPosition.getstart_dist()
				+ this.startPosition.getend_dist());
	}

	// trace back from aim box to original start point according to father and
	// generate a path
	public int[] generatePath() {
		PosWgt temp;
		for (temp = goal_inOpen; temp != startPosition; temp = temp.getFather()) {
			pa.push(temp.getFather().dir_calc(temp));
		}

		int i = 0;
		path = new int[pa.size()];

		while (!pa.empty()) {
			path[i] = pa.pop().intValue();
			i++;
		}

		return path;
	}

	// start to look for the minimum cost path
	public int[] startToSearch() {
		reset();

		// put the start box to the openlist
		getopenList().add(getstartPosition());

		// modify openlist and closelist
		attemptMove();
		path = generatePath();
		return path;
	}

	// set map

	// reset openlist and closelist
	private void reset() {
		// clear openlist
		getopenList().clear();

		// clear close list
		getcloseList().clear();
	}

	// find aim box in openlist
	public PosWgt findgoal_inOpen() {
		for (PosWgt a : openList) {
			if (a.cur_pos.x == getendPosition().cur_pos.x
					&& a.cur_pos.y == getendPosition().cur_pos.y) {
				return a;
			}
		}
		return null;
	}

	private void attemptMove() {
		// if aim box is in openlist or if openlist is empty, search fails
		if (box_exist_inOpen(getendPosition()) || getopenList().isEmpty()) {
			goal_inOpen = findgoal_inOpen();
			closeList.add(getendPosition());
			return;
		}

		// get box with minimum cost
		PosWgt minPosWgt = getMinPosWgt();

		// remove box with minimum cost from openlist
		getopenList().remove(minPosWgt);

		// try all reacheable points from minimum cost box
		List<PosWgt> subPosWgts = getReachableSubPositions(minPosWgt);

		// put all possible boxes in oplenlist
		for (PosWgt subPosWgt : subPosWgts) {
			addPosWgt(minPosWgt, subPosWgt);
		}

		getcloseList().add(minPosWgt);
		// repeat until succeed or permanent fail
		attemptMove();
	}

	// find out if a box is in openlist
	private boolean box_exist_inOpen(PosWgt box) {
		for (PosWgt a : openList) {
			if (a.cur_pos.x == box.cur_pos.x && a.cur_pos.y == box.cur_pos.y)
				return true;
		}
		return false;
	}

	// find out if a box is in closelist
	private boolean box_exist_inClose(PosWgt box) {
		for (PosWgt a : closeList) {
			if (a.cur_pos.x == box.cur_pos.x && a.cur_pos.y == box.cur_pos.y)
				return true;
		}
		return false;
	}

	// test reacheable boxes from current location
	private List<PosWgt> getReachableSubPositions(PosWgt father) {
		List<PosWgt> subPosWgts = new ArrayList<PosWgt>();

		PosCalc fatherPosition = father.getPosCalc();
		PosWgt subPosWgt = null;
		PosCalc subPosition = null;
		for (PosCalc offset : offsets) {
			subPosition = fatherPosition.offset(offset);
			subPosWgt = new PosWgt(subPosition, father, getstartPosition(),
					getendPosition());

			// if obstacle or unknown, or in closelist, continue
			if (agt.isObstacle(subPosition) || agt.isunKnown(subPosition)
					|| box_exist_inClose(subPosWgt)
					|| box_exist_inOpen(subPosWgt)) {
				continue;
			}

			subPosWgts.add(subPosWgt);
		}
		return subPosWgts;
	}

	// add a node
	private void addPosWgt(PosWgt father, PosWgt PosWgt) {
		// if already in openlist, update father and weight
		if (box_exist_inOpen(PosWgt)) {
			updateCostByFather(father, PosWgt);
		} else {
			getopenList().add(PosWgt);
		}
	}

	// find the box with minimum cost, if there are same cost boxes, choose the
	// one by random
	private PosWgt getMinPosWgt() {
		List<PosWgt> minPos = new ArrayList<PosWgt>();
		PosWgt minPosWgt = getopenList().get(0);
		for (PosWgt PosWgt : getopenList()) {
			if (minPosWgt.getCost() >= PosWgt.getCost()) {
				minPosWgt = PosWgt;
			}
		}
		for (PosWgt PosWgt : getopenList()) {
			if (minPosWgt.getCost() == PosWgt.getCost())
				minPos.add(PosWgt);
		}
		Random r = new Random(10);
		int i = r.nextInt(minPos.size());
		return minPos.get(i);
	}

	// if the new g value is smaller, update father box and g value
	private void updateCostByFather(PosWgt father, PosWgt subPosition) {
		int dist_father = subPosition.get_dist_father(father);
		int distanceOfstart = father.getstart_dist() + dist_father;
		if (distanceOfstart < subPosition.getstart_dist()) {
			subPosition.update_father(father);
		}
	}

	private List<PosWgt> getopenList() {
		return openList;
	}

	private List<PosWgt> getcloseList() {
		return closeList;
	}

	private PosWgt getendPosition() {
		return endPosition;
	}

	private PosWgt getstartPosition() {
		return startPosition;
	}
}