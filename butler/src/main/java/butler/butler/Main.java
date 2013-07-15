package butler.butler;

import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import move_base_msgs.MoveBaseActionGoal;

public class Main implements NodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("butler_main");
  }

  @Override
  public void onStart(ConnectedNode node) {

	Publisher<MoveBaseActionGoal> goal = node.newPublisher("move_base/goal", MoveBaseActionGoal._TYPE);

	try {
		Thread.sleep(1000);
	}
	catch (InterruptedException e) {
		e.printStackTrace();
	}

	MoveBaseActionGoal goalMsg = goal.newMessage();

	goalMsg.getGoal().getTargetPose().getHeader().setFrameId("base_link");
	goalMsg.getGoal().getTargetPose().getPose().getPosition().setX(7.0);
//	goalMsg.getGoal().getTargetPose().getPose().getPosition().setY(1.0);
	goalMsg.getGoal().getTargetPose().getPose().getOrientation().setW(1.0);
	goal.publish(goalMsg);

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
