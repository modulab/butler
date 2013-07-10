import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import sensor_msgs.JointState;

public class InitPTU implements NodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("init_ptu");
  }

  @Override
  public void onStart(ConnectedNode node) {

	Publisher<JointState> ptuPub = node.newPublisher("ptu/cmd", JointState._TYPE);

	try {
		Thread.sleep(13000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	JointState ptuCmd = ptuPub.newMessage();
	ptuCmd.getName().add("head_pan_joint");
	ptuCmd.getName().add("head_tilt_joint");

/*	ptuCmd.setPosition(new double[] {0,-0.8});
	ptuCmd.setVelocity(new double[] {1,1});
	ptuPub.publish(ptuCmd);
	try {
		Thread.sleep(3000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	ptuCmd.setPosition(new double[] {0,0.5});
	ptuPub.publish(ptuCmd);

	try {
		Thread.sleep(3000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	ptuCmd.setPosition(new double[] {2.5,0});
	ptuPub.publish(ptuCmd);
	try {
		Thread.sleep(3000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	ptuCmd.setPosition(new double[] {-2.5,0});
	ptuPub.publish(ptuCmd);

	try {
		Thread.sleep(3000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	} */

	ptuCmd.setPosition(new double[] {0,-0.81});
	ptuCmd.setVelocity(new double[] {1,1});
	ptuPub.publish(ptuCmd);

	try {
		Thread.sleep(1000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	ptuCmd.setPosition(new double[] {0,-0.82});
	ptuCmd.setVelocity(new double[] {1,1});
	ptuPub.publish(ptuCmd);
  }

  @Override
  public void onShutdown(Node node) {
  }

  @Override
  public void onShutdownComplete(Node node) {
  }

  @Override
  public void onError(Node node, Throwable throwable) {
  }
}
