import java.util.*;

public class movement {
	
	/*
	 * Movement method for use with the learning algorithms
	 * 
	 * Checks the proposed slope of the line in terms of 'y'/'x', then hands it off to the version of Bresenham's algorithm suited to handle that case
	 * 
	 * This returns a movementOutcome object, which contains the final position the car lands at after the move is completed, a boolean to note whether a collision occurred, 
	 * the new speed of the car, and the spaces traversed along the way to the final spot.
	 * 
	 * Note that all methods in movement class assume a track is surrounded on all sides by walls; if not, 'index out of bounds' errors might occur
	 */
	public static moveOutcome move(int x1, int y1, int x2, int y2, String[][] track, boolean restart, int startX, int startY) {
		double m = (double)(y2-y1)/(x2-x1);
		moveOutcome outcome;
		//Slopes that satisfy |m|<1
		if(Math.abs(m)<=1.0) {
			if(x1<x2) {
				outcome=lateralLR(x1,y1,x2,y2,track);
			}
			else {
				outcome=lateralRL(x1,y1,x2,y2,track);
			}
		}
		//Slopes that are |m|>1
		else {
			if(y1<y2) {
				outcome=verticalDown(x1,y1,x2,y2,track);
			}
			else {
				outcome=verticalUp(x1,y1,x2,y2,track);
			}
		}
		
		//If a collision occurred and we're using the case where the car is reset on collision, return the starting coordinates
		if(outcome.collision==true && restart) {
			outcome.x=startX;
			outcome.y=startY;
		}
		
		return outcome;
	}
	
	/*
	 * Method for simply testing movement alone
	 */
	public static moveOutcome move(int x1, int y1, int x2, int y2, String[][] track) {
		double m = (double)(y2-y1)/(x2-x1);
		moveOutcome outcome;
		if(Math.abs(m)<=1.0 && Double.isNaN(m)==false) {
			if(x1<x2) {
				outcome=lateralLR(x1,y1,x2,y2,track);
			}
			else {
				outcome=lateralRL(x1,y1,x2,y2,track);
			}
		}
		else {
			if(y1<y2) {
				outcome=verticalDown(x1,y1,x2,y2,track);
			}
			else {
				outcome=verticalUp(x1,y1,x2,y2,track);
			}
		}
		return outcome;
	}
	
	/*
	 * The four following methods implement Bresenham's algorithm on specific cases:
	 * 
	 * When the move is low |slope|, from left to right
	 * When the move is low |slope|, from right to left
	 * When the move is high |slope|, moving upwards
	 * When the move is high |slope|, moving downwards
	 * 
	 * Note that x's and y's may not accurately reflect conventional x and y dimensionalities (ie they are probably switched),
	 * and likewise lateral methods might actually be vertical and vice versa. Nonetheless, these methods work regardless.
	 * 
	 * All methods work fundamentally the same; determine where the car should go using a version of Bresenham's algorithm.
	 * newX and newY variables serve to preserve the original x or y should a collision happen
	 * 
	 */
	public static moveOutcome lateralLR(int x1, int y1, int x2, int y2, String[][] track) {
		double m = (double)(y2-y1)/(x2-x1);
		double lineY = y1;
		ArrayList<int[]> steps = new ArrayList<int[]>();
		for(int x=x1+1,y=y1;x<=x2;x++) {
			lineY+=m;
			int newY=y;
			double slope_error=lineY-(double)y;
			//Check whether y needs to be incremented or decremented
			if(m>0) {	
				if(slope_error>=0.5) {
					newY++;
				}
			}
			else {
				if(slope_error<=-0.5) {
					newY--;
				}
			}
			switch(checkCollision(x,newY,track)) {
			case 1:	//Collision
				return new moveOutcome(x-1,y,true,steps,5,5);
			case 2:	//Finish line
				return new moveOutcome(x,newY,false,steps,5,5);
			case 0:	//Regular space, continue
				y=newY;
				steps.add(new int[]{x,y});
			}
		}
		if(steps.size()>0) {
			steps.remove(steps.size()-1);
		}
		return new moveOutcome(x2,y2,false,steps,x2-x1+5,y2-y1+5);
	}
	
	public static moveOutcome lateralRL(int x1, int y1, int x2, int y2, String[][] track) {
		double m = (double)(y2-y1)/(x2-x1);
		double lineY = y1;
		ArrayList<int[]> steps = new ArrayList<int[]>();
		for(int x=x1-1,y=y1;x>=x2;x--) {
			lineY-=m;
			int newY=y;
			double slope_error=lineY-(double)y;
			//Check whether y needs to be incremented or decremented
			if(m<0) {	
				if(slope_error>=0.5) {
					newY++;
				}
			}
			else {
				if(slope_error<=-0.5) {
					newY--;
				}
			}
			switch(checkCollision(x,newY,track)) {
			case 1:	//Collision
				return new moveOutcome(x+1,y,true,steps,5,5);
			case 2:	//Finish line
				return new moveOutcome(x,newY,false,steps,5,5);
			case 0:	//Regular space, continue
				y=newY;
				steps.add(new int[]{x,y});
			}
		}
		if(steps.size()>0) {
			steps.remove(steps.size()-1);
		}
		return new moveOutcome(x2,y2,false,steps,x2-x1+5,y2-y1+5);
	}
	
	/*
	 * Methods with the axes flipped (i.e. slope='x'/'y' rather than 'y'/'x')
	 */
	public static moveOutcome verticalDown(int x1, int y1, int x2, int y2, String[][] track) {
		double m = (double)(x2-x1)/(y2-y1);
		double lineX = x1;
		ArrayList<int[]> steps = new ArrayList<int[]>();
		for(int x=x1,y=y1+1;y<=y2;y++) {
			lineX+=m;
			int newX=x;
			double slope_error=lineX-(double)x;
			//Check whether x needs to be incremented or decremented
			if(m>0) {
				if(slope_error>=0.5) {
					newX++;
				}
			}
			else {
				if(slope_error<=-0.5) {
					newX--;
				}
			}
			switch(checkCollision(newX,y,track)) {
			case 1:	//Collision
				return new moveOutcome(x,y-1,true,steps,5,5);
			case 2:	//Finish line
				return new moveOutcome(newX,y,false,steps,5,5);
			case 0:	//Regular space, continue
				x=newX;
				steps.add(new int[]{x,y});
			}
		}
		if(steps.size()>0) {
			steps.remove(steps.size()-1);
		}
		return new moveOutcome(x2,y2,false,steps,x2-x1+5,y2-y1+5);
	}
	
	public static moveOutcome verticalUp(int x1, int y1, int x2, int y2, String[][] track) {
		double m = (double)(x2-x1)/(y2-y1);
		double lineX = x1;
		ArrayList<int[]> steps = new ArrayList<int[]>();
		for(int x=x1, y=y1-1;y>=y2;y--) {
			lineX-=m;
			int newX=x;
			double slope_error=lineX-(double)x;
			//Check whether x needs to be incremented or decremented
			if(m<0) {	
				if(slope_error>=0.5) {
					newX++;
				}
			}
			else {
				if(slope_error<=-0.5) {
					newX--;
				}
			}
			switch(checkCollision(newX,y,track)) {
			case 1:	//Collision
				return new moveOutcome(x,y+1,true,steps,5,5);
			case 2:	//Finish line
				return new moveOutcome(newX,y,false,steps,5,5);
			case 0:	//Regular space, continue
				x=newX;
				steps.add(new int[]{x,y});
			}
		}
		if(steps.size()>0) {
			steps.remove(steps.size()-1);
		}
		return new moveOutcome(x2,y2,false,steps,x2-x1+5,y2-y1+5);
	}
	
	/*
	 * Method to check if a collision has occurred or if the car has reached the finish line
	 */
	public static int checkCollision(int x, int y, String[][] track) {
		switch(track[x][y]) {
		case "#":
			return 1;
		case "F":
			return 2;
		default:
			return 0;
		}
	}
	
	/*
	 * Method to determine where to place car, assuming the worst possible start (could help to disincentivize collisions a little extra)
	 */
	public static int[] determineWorstStart(String[][] track, double[][][][] states) {
		ArrayList<int[]> startingPositions = new ArrayList<int[]>();
		for(int x=0;x<track.length;x++) {
			for(int y=0;y<track.length;y++) {
				if(track[x][y].equals("S")) {
					startingPositions.add(new int[] {x,y});
				}
			}
		}
		int[] start = null;
		for(int[] candidate:startingPositions) {
			if(start==null||states[candidate[0]][candidate[1]][5][5]<states[start[0]][start[1]][5][5]) {
				start = candidate;
			}
		}
		return start;
	}
	
	/*
	 * Method to return a random starting position
	 */
	public static int[] determineRandomStart(String[][] track) {
		ArrayList<int[]> startingPositions = new ArrayList<int[]>();
		for(int x=0;x<track.length;x++) {
			for(int y=0;y<track.length;y++) {
				if(track[x][y].equals("S")) {
					startingPositions.add(new int[] {x,y});
				}
			}
		}
		Collections.shuffle(startingPositions);
		return startingPositions.get(0);
	}
	
	/*
	 * Method to determine the distance from the finish line of each position on the track
	 * 
	 * Returns a list of lists of spots on the track, where each list of spots is indexed by its distance from the 
	 */
	public static ArrayList<ArrayList<int[]>> markDistance(String[][] track){
		ArrayList<int[]> currLayer = new ArrayList<int[]>();
		ArrayList<ArrayList<int[]>> spotsByDistance = new ArrayList<ArrayList<int[]>>();
		int[][] visited = new int[track.length][track[0].length];	//2D array to track visited spaces
		
		//Start by noting the finish line spots and marking them visited
		for(int x=0;x<track.length;x++) {
			for(int y=0;y<track[x].length;y++) {
				if (track[x][y].equals("F")){
					currLayer.add(new int[] {x,y});
					visited[x][y] = 1;
				}
			}
		}
		
		//End when there's no more spaces to visit
		while(currLayer.size()>0) {
			ArrayList<int[]> thisLayer = new ArrayList<int[]>();
			//For each node (i.e. space on the track) in the current layer, check every neighbor it has
			for(int[] node: currLayer) {
				for(int x=-1;x<2;x++) {
					for(int y=-1;y<2;y++) {
						int[] neighbor = new int[] {node[0]+x,node[1]+y};
						//If a neighbor is not a wall and is not yet visited, add to current layer
						if(!track[neighbor[0]][neighbor[1]].equals("#") && visited[neighbor[0]][neighbor[1]]==0) {
							visited[neighbor[0]][neighbor[1]] = 1;
							thisLayer.add(neighbor);
						}
					}
				}
			}
			//If this layer contains spaces, add the layer to the list
			if(thisLayer.size()>0) {
				spotsByDistance.add(thisLayer);
			}
			currLayer=thisLayer;
		}
		
		return spotsByDistance;
	};
	
	/*
	 * Method to start close to finish line and progressively bring starting line backward
	 */
	public static int[] progressiveStartLine(String[][] track, ArrayList<ArrayList<int[]>> spotsByDistance, int iteration, int maxIteration) {
		double iter = (double) iteration;
		double maxIter = (double) maxIteration;
		
		//Only undergo this process in the first 1/2 of iterations
		if(iter/maxIter<(0.5)) {
			double newMaxIter = Math.ceil(maxIteration*(0.5));
			//Fetch the list of spots with distance=(distance of furthest spot)*(proportion of 'first-half iterations' already spent), 
			//then return a random spot from that list
			ArrayList<int[]> layer = spotsByDistance.get((int)(spotsByDistance.size()*(iter/newMaxIter)));
			Collections.shuffle(layer);
			return layer.get(0);
		}
		//Last 1/2 iterations just start on the starting line
		else {
			return determineRandomStart(track);
		}
	}
	
	/*
	 * Method to check if an acceleration is legal given the speed limits of {-5,5}
	 */
	public static boolean validAccel(int xSpd, int ySpd, int xAccel, int yAccel) {
		return !((xSpd==-5 && xAccel==-1) || (xSpd==5 && xAccel==1) || (ySpd==-5 && yAccel==-1) || (ySpd==5 && yAccel==1));
	}
}
