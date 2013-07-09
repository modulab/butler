import org.ros.namespace.GraphName;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import std_msgs.Bool;

public class UseKinectLaserScan implements NodeMain {

  @Override
  public GraphName getDefaultNodeName() {
    return GraphName.of("use_kinect_laser_scan");
  }

  @Override
  public void onStart(ConnectedNode node) {

	Publisher<Bool> pub = node.newPublisher("use_kinect_laser_scan", Bool._TYPE);
	Bool msg = pub.newMessage();
	msg.setData(true);

	while(true) {

		try {
			Thread.sleep(20);
		}
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	
		pub.publish(msg);
	}
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
