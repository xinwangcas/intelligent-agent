package xiw412;

//calculate position information
public class PosCalc {
	private final static int STRAIGHT = 2;// cost of moving forward is 2

	public int x;
	public int y;

	public PosCalc(int x, int y) {
		this.x = x;
		this.y = y;
	}

	// calculate manhatten distance to a certain box
	public int dist(PosCalc pos) {
		int verticalDistance = Math.abs(getY() - pos.getY());
		int horizontalDistance = Math.abs(getX() - pos.getX());
		return (verticalDistance + horizontalDistance) * STRAIGHT;
	}

	public PosCalc offset(PosCalc offset) {
		return new PosCalc(getX() + offset.getX(), getY() + offset.getY());
	}

	public int getX() {
		return x;
	}

	public void setX(int x) {
		this.x = x;
	}

	public int getY() {
		return y;
	}

	public void setY(int y) {
		this.y = y;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		PosCalc other = (PosCalc) obj;
		if (x != other.x)
			return false;
		if (y != other.y)
			return false;
		return true;
	}
}