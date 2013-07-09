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

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_read_goals");
	}

	public void onStart(ConnectedNode node) {
		while (true) {
			try {
				BufferedReader in = new BufferedReader(new InputStreamReader(new URL("http://localhost/Commands.txt").openStream()));
				String line = in.readLine();
				int currentSequenceNumber = Integer.parseInt(line.split(" ")[0]);

				if (currentSequenceNumber > lastSequenceNumber) {
					newGoal(node, Integer.parseInt(line.split(" ")[1]));
				}
				lastSequenceNumber = currentSequenceNumber;

				Thread.sleep(5000);
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}

	private void newGoal(ConnectedNode node, int goalID) {
		Publisher<Int32> intPub = node.newPublisher("qr_markers/goal", Int32._TYPE);

		Int32 msg = intPub.newMessage();
		msg.setData(goalID);
		intPub.publish(msg);
	}
}
