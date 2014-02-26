package xiw412;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// this class helps a node which is surrounded by fully aware situation to find a box
//which has never been visited before
public class BFS {

	private boolean SUCCESS = false;
	private PacAgent agt;


	// unvisted boxes that are to be visited soon
	private List<PosCalc> openLst;

	// visited boxes
	private List<PosCalc> closeLst;

	// posistion of the unvisited box the agent plan to go next
	private PosCalc endPos = null;

	// position of the box the agent starts in the round
	private PosCalc startPos;

	// offset of one move
	private List<PosCalc> offsets = Arrays.asList(new PosCalc[] {
			new PosCalc(1, 0), new PosCalc(0, 1), new PosCalc(-1, 0),
			new PosCalc(0, -1) });

	// constructor
	public BFS(PacAgent agt) {
		this.agt = agt;

		this.openLst = new ArrayList<PosCalc>();
		this.closeLst = new ArrayList<PosCalc>();
	}

	public List<PosCalc> getopenLst() {
		return openLst;
	}

	public List<PosCalc> getcloseLst() {
		return closeLst;
	}

	public PosCalc getstartPos() {
		return startPos;
	}

	public PosCalc getendPos() {
		return endPos;
	}

	public void setstartPos(PosCalc startPos) {
		this.startPos = startPos;
	}

	public void setendPos(PosCalc endPos) {
		this.endPos = endPos;
	}

	public void startToSearch() {
		reset();

		// put the start box to the openLst
		getopenLst().add(getstartPos());

		// modify openLst and closeLst
		breadth_first_search();

	}

	// reset openLst and closeLst
	private void reset() {
		// clear openLst
		getopenLst().clear();

		// clear close list
		getcloseLst().clear();

	}

	// get all reachable points from the current head node in openlist
	private List<PosCalc> getReachableSubPositions(PosCalc head) {
		List<PosCalc> children = new ArrayList<PosCalc>();
		for (PosCalc offset : offsets) {
			if (agt.isOutofbound(head.offset(offset)))// if the box is outofbound
			{
				continue;
			}
/*			if(agt.drop_x !=-1 && agt.drop_y !=-1){
				if(offset.x == agt.drop_x && offset.y == agt.drop_y){
					agt.drop_x = -1;
					agt.drop_y = -1;
					continue;
				}
			}*/

			children.add(head.offset(offset));
		}
		return children;
	}

	private boolean isInCloseTable(PosCalc pos) {
		return getcloseList().contains(pos);
	}

	private boolean isInOpenTable(PosCalc pos) {
		return getopenList().contains(pos);
	}

	private List<PosCalc> getopenList() {
		return openLst;
	}

	private List<PosCalc> getcloseList() {
		return closeLst;
	}

	private void breadth_first_search() {
		// if aim box is in openLst or if openLst is empty, search fails
		while (!openLst.isEmpty() && !SUCCESS) {
			// head from openLst
			PosCalc head = openLst.get(0);
			openLst.remove(head);

			if (agt.isunKnown(head)) {
				setendPos(head);
				SUCCESS = true;
				break;
			} else if (agt.isObstacle(head)){
				for(int i = 0; i < agt.tabPackages.size(); ++i){
					if(agt.tabPackages.get(i).y == head.y && agt.tabPackages.get(i).x == head.x){
						setendPos(head);
						SUCCESS = true;
						break;
					}
				}
			}
			
			// try all reacheable points from minimum cost box
			List<PosCalc> subPosCalcs = getReachableSubPositions(head);

			for (PosCalc pos : subPosCalcs) {
				if (!isInCloseTable(pos) && !isInOpenTable(pos) && !agt.isOutofbound(pos)) {// if already
																	// in
																	// openlist
																	// or in
																	// closelist,
																	// do not
																	// add to
																	// openlist
																	// again
					openLst.add(pos);
					closeLst.add(pos);
				}
			}
		}
	}
}
