public class valueIteration {
	String[][] track;
	double[][][][] states;
	boolean restart;
	double decay;
	
	/*
	 * Method to train with value iteration
	 */
	public void train(String[][] track, double bellmanError, int maxIterations, boolean restart, double decay) {
		this.states = new double[track.length][track[0].length][11][11];
		this.track = track;
		this.restart = restart;
		this.decay=decay;
		
		for(int iterations=0;iterations<maxIterations;iterations++) {
			int[] start = movement.determineWorstStart(this.track,this.states);
			double maxDelta=0;
			double[][][][] lastIter = copyArray(states);
			//Iterate over states
			for(int xPos=0;xPos<states.length;xPos++) {
				for(int yPos=0;yPos<states[xPos].length;yPos++) {
					
					if( !(track[xPos][yPos].equals("#") || track[xPos][yPos].equals("F"))) {
						
						for(int xSpd=0;xSpd<states[xPos][yPos].length;xSpd++) {
							for(int ySpd=0;ySpd<states[xPos][yPos][xSpd].length;ySpd++) {
								//Calculate true speeds from indices
								int trueXSpd = xSpd-5;
								int trueYSpd = ySpd-5;
								//Initialize Qmax to infinitely small value
								double maxQ = -1.0*Double.MAX_VALUE;
								//Iterate over actions
								for(int xAccel=0;xAccel<3;xAccel++) {
									for(int yAccel=0;yAccel<3;yAccel++) {
										//Calculate true accel from indices
										int trueXAccel = xAccel-1;
										int trueYAccel = yAccel-1;
										//If the action is a valid action, calculate Q(s,a)
										if(movement.validAccel(trueXSpd,trueYSpd,trueXAccel,trueYAccel)) {
											//Calculate value if acceleration works
											double desiredValue=0;
											moveOutcome desiredOutcome = movement.move(xPos,yPos,(xPos+trueXSpd+trueXAccel),(yPos+trueYSpd+trueYAccel),track, restart,start[0], start[1]);
											desiredValue = lastIter[desiredOutcome.x][desiredOutcome.y][desiredOutcome.xSpd][desiredOutcome.ySpd];
											
											//Calculate value if acceleration doesn't work
											double undesiredValue =0;
											moveOutcome undesiredOutcome = movement.move(xPos,yPos,(xPos+trueXSpd),(yPos+trueYSpd),track, restart, start[0], start[1]);
											undesiredValue = lastIter[undesiredOutcome.x][undesiredOutcome.y][undesiredOutcome.xSpd][undesiredOutcome.ySpd];
											
											//Calculate Q(s,a)
											double Q = (0.8*determineReward(track,desiredOutcome)+0.2*determineReward(track,undesiredOutcome))+decay*(0.8*desiredValue+0.2*undesiredValue);
											
											//update the maxQ if applicable
											if(Q>maxQ) {
												maxQ=Q;
											}
										}
									}
								}
								//Set the value for V(s) from max Q-value
								states[xPos][yPos][xSpd][ySpd] = maxQ;
								//Check for new max delta
								double delta = Math.abs(states[xPos][yPos][xSpd][ySpd] - lastIter[xPos][yPos][xSpd][ySpd]);
								if (delta>maxDelta) {
									maxDelta=delta;
								}
							}
						}
					}
				}
			}
			//Print updates on 10th iteration
			if(iterations==10) {
				for(int x=0;x<track.length;x++) {
					for(int y=0;y<track[x].length;y++) {
						if(!track[x][y].equals("#")) {
							System.out.println(x + "," + y + " (speed 1,1): " + lastIter[x][y][6][6] + "<- " + states[x][y][6][6]);
						}
					}
				}
			}
			
			if(maxDelta<bellmanError) {
				System.out.println(iterations + " iterations required.");
				break;
			}
		}
		
		test();
	}
	
	/*
	 * Method to test the model
	 */
	public void test() {
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
		
		while(!track[currentState[0]][currentState[1]].equals("F")) {
			//Calculate true speeds from indices
			int trueXSpd = currentState[2]-5;
			int trueYSpd = currentState[3]-5;
			
			double maxQ = -1.0*Double.MAX_VALUE;
			int[] optimalAccel = new int[] {0,0};
			
			for(int xAccel=0;xAccel<3;xAccel++) {
				for(int yAccel=0;yAccel<3;yAccel++) {
					//Calculate true accel from indices
					int trueXAccel = xAccel-1;
					int trueYAccel = yAccel-1;
					//If the action is a valid action, calculate Q(s,a)
					if(movement.validAccel(trueXSpd,trueYSpd,trueXAccel,trueYAccel)) {
						//Calculate the value if acceleration works
						double desiredValue=0;
						moveOutcome desiredOutcome = movement.move(currentState[0],currentState[1],(currentState[0]+trueXSpd+trueXAccel),(currentState[1]+trueYSpd+trueYAccel),track, restart, start[0], start[1]);
						desiredValue = states[desiredOutcome.x][desiredOutcome.y][desiredOutcome.xSpd][desiredOutcome.ySpd];
						
						//Calculate the value if acceleration doesn't work 
						double undesiredValue =0;
						moveOutcome undesiredOutcome = movement.move(currentState[0],currentState[1],(currentState[0]+trueXSpd),(currentState[1]+trueYSpd),track, restart, start[0], start[1]);
						undesiredValue = states[undesiredOutcome.x][undesiredOutcome.y][desiredOutcome.xSpd][desiredOutcome.ySpd];
						
						//Calculate Q(s,a)
						double Q = (0.8*determineReward(track,desiredOutcome)+0.2*determineReward(track,undesiredOutcome))+decay*(0.8*desiredValue+0.2*undesiredValue);
						
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
				System.out.println("Acceleration failed at " + currentState[0] + ", " + currentState[1]);
				outcome = movement.move(currentState[0],currentState[1],currentState[0]+trueXSpd,currentState[1]+trueYSpd,track, restart, start[0], start[1]);
			}
			
			//Mark the steps taken
			for(int[] intermediateStep:outcome.steps) {
				if(!movementTrack[intermediateStep[0]][intermediateStep[1]].equals("X")) {
					movementTrack[intermediateStep[0]][intermediateStep[1]]="+";
				}
			}
			movementTrack[outcome.x][outcome.y]="O";
			//Mark any collisions
			if(outcome.collision==true && restart) {
				int[] collisionPoint; 
				if(outcome.steps.size()>0) {
					collisionPoint= outcome.steps.get(outcome.steps.size()-1);
				}
				else {
					collisionPoint= new int[] {currentState[0],currentState[1]};
				}
				movementTrack[collisionPoint[0]][collisionPoint[1]]="X";
				System.out.println("Collision at (" + outcome.x + "," + outcome.y + ")");
			}
			else if(outcome.collision==true) {
				movementTrack[outcome.x][outcome.y]="X";
				System.out.println("Collision at (" + outcome.x + "," + outcome.y + ")");
			}
			
			currentState = new int[] {outcome.x,outcome.y,outcome.xSpd,outcome.ySpd};
			moves++;
		}
		//Print the movements made
		for(int x=0;x<track.length;x++) {
			for(int y=0;y<track[x].length;y++) {
				System.out.print(movementTrack[x][y]);
			}
			System.out.println();
		}
		System.out.println("Moves required: " + moves);
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
	
	/*
	 * Method to clone the state array for checking Bellman error
	 */
	public static double[][][][] copyArray(double[][][][] states){
		double[][][][] clone = new double[states.length][states[0].length][11][11];
		
		for(int xPos=0;xPos<states.length;xPos++) {
			for(int yPos=0;yPos<states[xPos].length;yPos++) {
				for(int xSpd=0;xSpd<states[xPos][yPos].length;xSpd++) {
					for(int ySpd=0;ySpd<states[xPos][yPos][xSpd].length;ySpd++) {
						clone[xPos][yPos][xSpd][ySpd]=states[xPos][yPos][xSpd][ySpd];
					}
				}
			}
		}
		return clone;			
	}
}

