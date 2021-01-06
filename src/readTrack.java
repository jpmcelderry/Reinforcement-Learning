import java.io.*;

public class readTrack {
	/*
	 * Method to transform input track to 2D array
	 */
	public static String[][] read(String file) throws Exception {
		BufferedReader buffReader = null;
		String[][] stringArray;
		try {
			buffReader = new BufferedReader(new FileReader(file));
			String currentLine;
			//Read dimensions
			String[] dims = buffReader.readLine().split(",");
			stringArray = new String[Integer.parseInt(dims[0])][Integer.parseInt(dims[1])];
			//Read file to string array
			int lineCounter = 0;
			while((currentLine=buffReader.readLine()) != null) {	//read until end of file
				if(!currentLine.trim().equals("")) {	//don't read empty lines
					stringArray[lineCounter]=currentLine.split("");
				}
				lineCounter++;
			}
		}
		finally {
			if(buffReader != null) {buffReader.close();}
		}
		return stringArray;
	}
}
