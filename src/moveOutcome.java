import java.util.*;

/*
 * Helper class to store information on the outcome of a move
 */
public class moveOutcome {
	int x;
	int y;
	boolean collision;
	ArrayList<int[]> steps;
	int xSpd;
	int ySpd;
	
	
	moveOutcome(int x, int y, boolean collision, ArrayList<int[]> steps, int xSpd, int ySpd){
		this.x=x;
		this.y=y;
		this.collision=collision;
		this.steps=steps;
		this.xSpd=xSpd;
		this.ySpd=ySpd;
	}
}
