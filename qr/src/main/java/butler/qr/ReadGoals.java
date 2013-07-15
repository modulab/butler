package butler.qr;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.net.URL;

import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import std_msgs.Int32;

public class ReadGoals extends AbstractNodeMain {

	private int lastSequenceNumber = -1;
	private Publisher<Int32> intPub;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_read_goals");
	}

	public void onStart(ConnectedNode node) {

		intPub = node.newPublisher("qr_markers/goal", Int32._TYPE);

		while (true) {
			try {
				BufferedReader in = new BufferedReader(new InputStreamReader(new URL("http://localhost/Commands.txt").openStream()));
				String line = in.readLine();
				int currentSequenceNumber = Integer.parseInt(line.split(" ")[0]);

				if ((currentSequenceNumber > lastSequenceNumber) && (lastSequenceNumber != -1)) {
					while ((Integer.parseInt(line.split(" ")[1]) != lastSequenceNumber)) {
						if (in.ready()) {
							line = in.readLine();
						} else {
							throw new Exception("Previous sequence number not found");
						}
					}
					line = in.readLine();
					while (in.ready() && !(line.equals("EOF"))) {
						newGoal(node, Integer.parseInt(line.split(" ")[2]));
						line = in.readLine();
					}
				}
				lastSequenceNumber = currentSequenceNumber;

				Thread.sleep(5000);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private void newGoal(ConnectedNode node, int goalID) {
		Int32 msg = intPub.newMessage();
		msg.setData(goalID);
		intPub.publish(msg);
	}
}
