public class ML6 {
	public static void main(String[] args) throws Exception {
		for (String file: args) {
			String[][] track = readTrack.read(file);
			
			valueIteration vI = new valueIteration();
			vI.train(track,.000001,1000,false,0.7);
			if(file.equals("R-track.txt")) {
				vI.train(track,.000001,1000,true,0.7);
			}
			
			QLearning Q = new QLearning();
			Q.train(track, 750000, false, 0.8, 0.5, 0.8);
			if(file.equals("R-track.txt")) {
				Q.train(track, 750000, true, 0.8, 0.5, 0.5);
			}
			
			SARSA sarsa = new SARSA();
			sarsa.train(track, 750000, false, 0.8, 0.5, 0.8);
			if(file.equals("R-track.txt")) {
				sarsa.train(track, 750000, true, 0.8, 0.5, 0.5);
			}
			
		}
	}
}
