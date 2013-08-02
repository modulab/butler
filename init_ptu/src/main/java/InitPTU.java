import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;

import sensor_msgs.JointState;

public class InitPTU extends AbstractNodeMain {

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("butler/init_ptu");
	}

	@Override
	public void onStart(ConnectedNode node) {

		Publisher<JointState> ptuPub = node.newPublisher("ptu/cmd", JointState._TYPE);

		try {
			Thread.sleep(13000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		JointState ptuCmd = ptuPub.newMessage();
		ptuCmd.getName().add("head_pan_joint");
		ptuCmd.getName().add("head_tilt_joint");
		ptuCmd.setPosition(new double[] { 0, -0.30 });
		ptuCmd.setVelocity(new double[] { 1, 1 });
		//ptuPub.publish(ptuCmd);

		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		ptuCmd.setPosition(new double[] { 0, -0.82 });
//		ptuCmd.setPosition(new double[] { 0, -0.60 });
		ptuCmd.setVelocity(new double[] { 1, 1 });
		ptuPub.publish(ptuCmd);
	}
}
