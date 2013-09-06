package butler.qr;

import java.io.File;
import java.io.IOException;
import java.io.StringReader;
import java.util.List;
import java.util.ArrayList;

import move_base_msgs.MoveBaseActionGoal;

import org.apache.commons.logging.Log;
import org.jdom2.Document;
import org.jdom2.Element;
import org.jdom2.JDOMException;
import org.jdom2.input.SAXBuilder;
import org.jdom2.input.sax.XMLReaders;
import org.ros.exception.RemoteException;
import org.ros.exception.ServiceException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.xml.sax.EntityResolver;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import std_msgs.Bool;
import std_msgs.Int32;
import visualization_msgs.InteractiveMarkerFeedback;
import visualization_msgs.InteractiveMarkerInit;
import web_connector.GetActiveOrdersRequest;
import web_connector.GetActiveOrdersResponse;
import web_connector.GetOrders;
import web_connector.GetOrdersRequest;
import web_connector.GetOrdersResponse;
import web_connector.MarkActiveOrdersRequest;
import web_connector.MarkActiveOrdersResponse;
import web_connector.Order;

public class Markers extends AbstractNodeMain {

	private InteractiveMarkerInit currentUpdate = null;
	private File locations = new File(
			"/home/sean/ROS/butler_workspace/butler/qr/locations.xml");
	private Publisher<MoveBaseActionGoal> goalPub, baseGoalPub;
	private Publisher<std_msgs.String> feedbackPub;
	private Publisher<Bool> resultPub;
	private Log log;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_markers");
	}

	@Override
	public void onStart(ConnectedNode node) {
		log = node.getLog();

		baseGoalPub = node.newPublisher("butler/base_goal",
				MoveBaseActionGoal._TYPE);
		baseGoalPub.setLatchMode(true);
		
		feedbackPub = node.newPublisher("crowded_nav/feedback",
				std_msgs.String._TYPE);
		resultPub = node.newPublisher("crowded_nav/result", Bool._TYPE);

		goalPub = node.newPublisher("butler/goal", MoveBaseActionGoal._TYPE);

		Subscriber<InteractiveMarkerInit> markerUpdateSub = node.newSubscriber(
				"marker_server/update_full", InteractiveMarkerInit._TYPE);

		markerUpdateSub
				.addMessageListener(new MessageListener<InteractiveMarkerInit>() {
					@Override
					public void onNewMessage(InteractiveMarkerInit update) {
						currentUpdate = update;
					}
				});

		Subscriber<Int32> goSub = node.newSubscriber(
				"crowded_nav/go", Int32._TYPE);

		goSub.addMessageListener(new MessageListener<Int32>() {
			@Override
			public void onNewMessage(Int32 update) {
				feedback("Moving to station "+update.getData());
				sendGoalMessage(update.getData());
			}
		});

		readXML(node);

		try {
			Thread.sleep(15000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		sendGoalMessage(0, true);

	}

	private void readXML(ConnectedNode node) {
		if (locations.exists()) {
			Document doc = null;
			SAXBuilder builder = new SAXBuilder(XMLReaders.NONVALIDATING);
			builder.setEntityResolver(new EntityResolver() {
				@Override
				public InputSource resolveEntity(String publicId,
						String systemId) throws SAXException, IOException {
					return new InputSource(new StringReader(""));
				}
			});

			try {
				doc = builder.build(locations);
			} catch (JDOMException ex) {
				System.err
						.println(locations.getAbsolutePath()
								+ " is either not a well-formed XML document or is not valid: "
								+ ex.getMessage());
			} catch (IOException ex) {
				ex.printStackTrace();
			}

			final Publisher<InteractiveMarkerFeedback> markerPub = node
					.newPublisher("marker_server/feedback",
							InteractiveMarkerFeedback._TYPE);

			System.out.println("!!!!! "
					+ doc.getRootElement().getChildren().size());
			List<Element> children = doc.getRootElement().getChildren();

			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}

			for (int i = 0; i < children.size(); i++) {
				InteractiveMarkerFeedback newMarker = markerPub.newMessage();

				newMarker.getHeader().setFrameId("map");
				newMarker.setMarkerName("point " + i);
				newMarker.setEventType(InteractiveMarkerFeedback.POSE_UPDATE);
				newMarker
						.getPose()
						.getPosition()
						.setX(Double.parseDouble(children.get(i).getChild("px")
								.getText()));
				newMarker
						.getPose()
						.getPosition()
						.setY(Double.parseDouble(children.get(i).getChild("py")
								.getText()));
				newMarker
						.getPose()
						.getPosition()
						.setZ(Double.parseDouble(children.get(i).getChild("pz")
								.getText()));
				newMarker
						.getPose()
						.getOrientation()
						.setX(Double.parseDouble(children.get(i).getChild("ox")
								.getText()));
				newMarker
						.getPose()
						.getOrientation()
						.setY(Double.parseDouble(children.get(i).getChild("oy")
								.getText()));
				newMarker
						.getPose()
						.getOrientation()
						.setZ(Double.parseDouble(children.get(i).getChild("oz")
								.getText()));
				newMarker
						.getPose()
						.getOrientation()
						.setW(Double.parseDouble(children.get(i).getChild("ow")
								.getText()));

				markerPub.publish(newMarker);
			}
		} else {
			System.out.println("Locations file does not exist.");
		}
	}

	private MoveBaseActionGoal sendGoalMessage(int id) {
		return sendGoalMessage(id, false);
	}

	private MoveBaseActionGoal sendGoalMessage(int id, boolean base) {
		MoveBaseActionGoal goalMsg = goalPub.newMessage();

		try {

			while (currentUpdate == null) {
				log.warn("Markers: Waiting for marker update...");
				Thread.sleep(100);
			}

			goalMsg.getGoal().getTargetPose().getHeader().setFrameId("map");

			boolean valid = false;

			for (int i = 0; i < currentUpdate.getMarkers().size(); i++) {
				if (currentUpdate.getMarkers().get(i).getName()
						.equals("point " + id)) {
					valid = true;
					goalMsg.getGoal()
							.getTargetPose()
							.setPose(
									currentUpdate.getMarkers().get(i).getPose());
				}
			}

			if (valid) {
				if (base) {
					baseGoalPub.publish(goalMsg);
					System.out.println("Sending base goal: ("
							+ goalMsg.getGoal().getTargetPose().getPose()
									.getPosition().getX()
							+ ","
							+ goalMsg.getGoal().getTargetPose().getPose()
									.getPosition().getY());
				} else {
					goalPub.publish(goalMsg);
				}
			} else {
				feedback("Failed to find marker " + id);
				result(false);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		return goalMsg;
	}
	
	private void feedback(String message) {
		System.out.println(message);

		std_msgs.String feedbackMsg = feedbackPub.newMessage();
		feedbackMsg.setData(message);
		feedbackPub.publish(feedbackMsg);
	}
	
	private void result(boolean result) {
		Bool resultMsg = resultPub.newMessage();
		resultMsg.setData(result);
		resultPub.publish(resultMsg);
	}
}
