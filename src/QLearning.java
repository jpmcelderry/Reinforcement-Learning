import java.util.*;
import java.io.*;

public class QLearning {
	String[][] track;
	double[][][][][][] states;
	boolean restart;
	double decay;
	
	/*
	 * Method to train a Q-Learning model
	 */
	public void train(String[][] track, int maxIterations, boolean restart, double decay, double a, double epsilon) throws IOException {	
		this.states = new double[track.length][track[0].length][11][11][3][3];
		this.track = track;
		this.restart = restart;
		this.decay=decay;
		BufferedWriter buffWriter = new BufferedWriter(new FileWriter("QLearn-out.txt"));
		buffWriter.write("Iteration\tMoves\n");
		
		//Initialize all states other than the goal to random value
		for(int xPos=0;xPos<states.length;xPos++) {
			for(int yPos=0;yPos<states[xPos].length;yPos++) {
				if( track[xPos][yPos].equals(".") || track[xPos][yPos].equals("S")) {
					for(int xSpd=0;xSpd<states[xPos][yPos].length;xSpd++) {
						for(int ySpd=0;ySpd<states[xPos][yPos][xSpd].length;ySpd++) {
							//Loop over actions
							for(int xAccel=0;xAccel<3;xAccel++) {
								for(int yAccel=0;yAccel<3;yAccel++) {
									if(movement.validAccel(xSpd-5,ySpd-5,xAccel-1,yAccel-1)) {
										states[xPos][yPos][xSpd][ySpd][xAccel][yAccel] = -1.0*Math.random();
									}
								}
							}
						}
					}
				}
			}
		}
		
		//Calculate the distance of each track location from the finish line
		ArrayList<ArrayList<int[]>> distances = movement.markDistance(track);
		
		for(int iterations=0;iterations<maxIterations;iterations++) {
			boolean print = (iterations==550000);
			
			if((iterations+1)%10000==0) {
				System.out.println("Iteration " + (iterations+1) + "...");
			}
			
			//Initialize s; if restart is true, then start near finish line and back up progressively
			int[] start;
			if(restart) {
				start = movement.progressiveStartLine(track,distances,iterations,maxIterations);
			}
			else {
				start = movement.determineRandomStart(this.track);
			}
			int[] currentState = new int[]{start[0],start[1],5,5};
			
			//Code for one trial
			while(!track[currentState[0]][currentState[1]].equals("F")) {
				int trueXSpd = currentState[2]-5;
				int trueYSpd = currentState[3]-5;
				
				//Instatiate optimal accel to 0,0 (or 1,1 when represented as indices in the table)
				int[] accel = new int[] {1,1};
				double maxQ = states[ currentState[0] ][ currentState[1] ][ currentState[2] ][ currentState[3] ][1][1];
				ArrayList<int[]> legalMoves = new ArrayList<int[]>();

				//Iterate over acceleration options
				for(int xAccel=0;xAccel<3;xAccel++) {
					for(int yAccel=0;yAccel<3;yAccel++) {
						//If the action is a valid action
						if(movement.validAccel(trueXSpd,trueYSpd,xAccel-1,yAccel-1)) {
							//Retrieve Q(s,a)
							double Q = states[ currentState[0] ][ currentState[1] ][ currentState[2] ][ currentState[3] ][xAccel][yAccel];
							//update the maxQ if applicable
							if(Q>maxQ) {
								legalMoves.add(accel);
								maxQ=Q;
								accel = new int[] {xAccel,yAccel};
							}
							//Otherwise add to random options ({1,1} should either be the chosen accel or added to this list in the other case)
							else if (!(xAccel==1 && yAccel==1)){
								legalMoves.add(new int[] {xAccel,yAccel});
							}
						}
					}
				}
				
				//Check for exploration
				if(Math.random()<epsilon && legalMoves.size()>0) {
					Collections.shuffle(legalMoves);
					if(print) {	
						System.out.println("Optimal action was (" + (accel[0]-1) + "," + (accel[1]-1) + ") , took action (" + legalMoves.get(0)[0] + ", " + legalMoves.get(0)[1] + ")");
					}
					accel = legalMoves.get(0);
				}
				else {
					if(print) {	
						System.out.println("Optimal action (" + (accel[0]-1) + "," + (accel[1]-1) + ") was taken.");
					}
				}
				
				//Check for acceleration failure
				moveOutcome outcome;
				if(Math.random()>=0.2) {
					outcome = movement.move(currentState[0],currentState[1],currentState[0]+trueXSpd+(accel[0]-1),currentState[1]+trueYSpd+(accel[1]-1),track, restart, start[0], start[1]);
				}
				//Acceleration doesn't work
				else {
					outcome = movement.move(currentState[0],currentState[1],currentState[0]+trueXSpd,currentState[1]+trueYSpd,track, restart, start[0], start[1]);
				}
				
				//Determine Q(s,a)
				double originalQ = states[ currentState[0] ][ currentState[1] ][ currentState[2] ][ currentState[3] ][ accel[0] ][ accel[1] ];
				
				//Determine a'
				double nextQmax = states[outcome.x][outcome.y][outcome.xSpd][outcome.ySpd][1][1];;
				for(int xAccel=0;xAccel<3;xAccel++) {
					for(int yAccel=0;yAccel<3;yAccel++) {
						//Check valid moves
						if(movement.validAccel(outcome.xSpd-5,outcome.ySpd-5,xAccel-1,yAccel-1)) {
							//Check for new best move
							if(states[outcome.x][outcome.y][outcome.xSpd][outcome.ySpd][xAccel][yAccel]>nextQmax) {	
								nextQmax=states[outcome.x][outcome.y][outcome.xSpd][outcome.ySpd][xAccel][yAccel];
							}
						}
					}
				}
				
				//Apply update, print if applicable
				double update = originalQ*(1.0-a)+a*(determineReward(track,outcome)+decay*(nextQmax)-originalQ);
				if(print) {
					System.out.print( currentState[0] + "," + currentState[1] + "," + (currentState[2]+5) + "," + (currentState[3]+5) + "," + accel[0] + "," + accel[1]);
					System.out.print(": " + states[currentState[0]][currentState[1]][currentState[2]][currentState[3]][accel[0]][accel[1]] + "<- " + update + "\n");
				}
				states[ currentState[0] ][ currentState[1] ][ currentState[2] ][ currentState[3] ][ accel[0] ][ accel[1] ]=update;
				
				currentState = new int[] {outcome.x,outcome.y,outcome.xSpd,outcome.ySpd};
			}
			//decrement epsilon and a
			epsilon-=epsilon/(double)maxIterations;
			a-=a/(double)maxIterations;
			
			//Test the model at interesting increments
			if(iterations<10000 && (iterations+1)%1000==0) {
				int moves = test(false);
				buffWriter.write(iterations + "\t" + moves + "\n");
			}
			else if((iterations+1)%10000==0) {
				int moves = test(false);
				buffWriter.write(iterations + "\t" + moves + "\n");
			}
		}
		
		//Test the finished model
		test(true);
		buffWriter.close();
	}
	
	/*
	 * Method to test the model
	 */
	public int test(boolean print) {
		int[] start = movement.determineRandomStart(this.track);
		int[] currentState = new int[]{start[0],start[1],5,5};
		String[][] movementTrack = new String[track.length][track[0].length];
		
		//Create a dummy copy of the track for purposes of printing
		for(int x=0;x<track.length;x++) {
			for(int y=0;y<track[x].length;y++) {
				movementTrack[x][y] = track[x][y];
			}
		}
		movementTrack[start[0]][start[1]]="O";
		int moves=0;
		
		while(!track[currentState[0]][currentState[1]].equals("F") && moves<1000) {
			//Get true speeds from indices
			int trueXSpd = currentState[2]-5;
			int trueYSpd = currentState[3]-5;
			
			double maxQ = -1.0*Double.MAX_VALUE;
			int[] optimalAccel = new int[] {0,0};
			
			//Iterate over all possible actions
			for(int xAccel=0;xAccel<3;xAccel++) {
				int trueXAccel = xAccel-1;
				for(int yAccel=0;yAccel<3;yAccel++) {
					int trueYAccel = yAccel-1;
					//If the action is a valid action
					if(movement.validAccel(trueXSpd,trueYSpd,trueXAccel,trueYAccel)) {
						double Q = states[ currentState[0] ][ currentState[1] ][ currentState[2] ][ currentState[3] ][xAccel][yAccel];
						//update the maxQ if applicable
						if(Q>maxQ) {
							maxQ=Q;
							optimalAccel = new int[] {trueXAccel,trueYAccel};
						}
					}
				}
			}
			moveOutcome outcome;
			//Acceleration works
			if(Math.random()>=0.2) {
				outcome = movement.move(currentState[0],currentState[1],currentState[0]+trueXSpd+optimalAccel[0],currentState[1]+trueYSpd+optimalAccel[1],track, restart, start[0], start[1]);
			}
			//Acceleration doesn't work
			else {
				if(print) {
					System.out.println("Acceleration failed at " + currentState[0] + ", " + currentState[1]);
				}
				outcome = movement.move(currentState[0],currentState[1],currentState[0]+trueXSpd,currentState[1]+trueYSpd,track, restart, start[0], start[1]);
			}
			//Mark the steps taken
			for(int[] intermediateStep:outcome.steps) {
				if(!movementTrack[intermediateStep[0]][intermediateStep[1]].equals("X")) {
					movementTrack[intermediateStep[0]][intermediateStep[1]]="+";
				}
			}
			movementTrack[outcome.x][outcome.y]="O";
			//Mark collisions
			if(outcome.collision==true && restart) {
				int[] collisionPoint; 
				if(outcome.steps.size()>0) {
					collisionPoint= outcome.steps.get(outcome.steps.size()-1);
				}
				else {
					collisionPoint= new int[] {currentState[0],currentState[1]};
				}
				movementTrack[collisionPoint[0]][collisionPoint[1]]="X";
				if(print) {
					System.out.println("Collision at (" + outcome.x + "," + outcome.y + ")");
				}
			}
			else if(outcome.collision==true) {
				movementTrack[outcome.x][outcome.y]="X";
				if(print) {
					System.out.println("Collision at (" + outcome.x + "," + outcome.y + ")");
				}
			}
			//Update current state
			currentState = new int[] {outcome.x,outcome.y,outcome.xSpd,outcome.ySpd};
			moves++;
		}
		//Print the movements made
		if(print) {	
			for(int x=0;x<track.length;x++) {
				for(int y=0;y<track[x].length;y++) {
					System.out.print(movementTrack[x][y]);
				}
				System.out.println();
			}

			System.out.println("Moves required: " + moves);
		}
		
		return moves;
	}
	
	/*
	 * Method to provide reward
	 */
	public static double determineReward(String[][] track, moveOutcome move) {
		if(track[move.x][move.y].equals("F")) {
			return 0.0;
		}
		else {
			return -1.0;
		}
	}
}
