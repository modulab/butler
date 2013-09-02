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
	private Log log;
	private ConnectedNode node;
	private GetOrdersResponse getOrdersResponse;
	private GetActiveOrdersResponse getActiveOrdersResponse;
	private MarkActiveOrdersResponse markActiveOrdersResponse;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_markers");
	}

	@Override
	public void onStart(ConnectedNode node) {
		log = node.getLog();
		this.node = node;

		baseGoalPub = node.newPublisher("butler/base_goal",
				MoveBaseActionGoal._TYPE);
		baseGoalPub.setLatchMode(true);

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

		readXML(node);

		try {
			Thread.sleep(15000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		createGoalMessage(0, true);

		Subscriber<Int32> goalPointSub = node.newSubscriber("qr_markers/goal",
				Int32._TYPE);
		goalPointSub.addMessageListener(new MessageListener<Int32>() {

			@Override
			public void onNewMessage(Int32 goalPoint) {
				createGoalMessage(goalPoint.getData());
			}
		});

		node.newServiceServer(
				"qr/get_goal",
				GetOrders._TYPE,
				new ServiceResponseBuilder<GetOrdersRequest, GetOrdersResponse>() {

					@Override
					public void build(GetOrdersRequest arg0,
							GetOrdersResponse arg1) throws ServiceException {

						List<Order> orders = getOrders();
						List<String> activeOrders = getActiveOrders();

						if (activeOrders.size() == 0) {
							activeOrders = markActiveOrders();
						}
						boolean failed = true;
						if (activeOrders.size() > 0) {
							for (Order o : orders) {
								if (o.getOrderId() == activeOrders.get(0)) {
									o.setGoal(createGoalMessage(Integer.parseInt(o.getStationId())));
									arg1.getOrders().add(o);
									failed = false;
								}
							}

							if (failed == true) {
								log.error("Failed to find active order in order list");
							}
						} else {
							log.info("No active orders");
						}
					}
				});

	}

	private List<Order> getOrders() {
		ServiceClient<GetOrdersRequest, GetOrdersResponse> getOrdersClient = null;
		try {
			getOrdersClient = node.newServiceClient(
					"RosWebInterface/get_orders", Order._TYPE);
		} catch (Exception e) {
		}
		getOrdersClient.call(getOrdersClient.newMessage(),
				new ServiceResponseListener<GetOrdersResponse>() {

					@Override
					public void onSuccess(GetOrdersResponse arg0) {
						getOrdersResponse = arg0;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						log.error("Failed to get orders");
					}
				});
		return getOrdersResponse.getOrders();
	}

	private List<String> getActiveOrders() {
		ServiceClient<GetActiveOrdersRequest, GetActiveOrdersResponse> getActiveOrdersClient = null;
		try {
			getActiveOrdersClient = node.newServiceClient(
					"RosWebInterface/get_active_orders", Order._TYPE);
		} catch (Exception e) {
		}
		getActiveOrdersClient.call(getActiveOrdersClient.newMessage(),
				new ServiceResponseListener<GetActiveOrdersResponse>() {

					@Override
					public void onSuccess(GetActiveOrdersResponse arg0) {
						getActiveOrdersResponse = arg0;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						log.error("Failed to get active orders");
					}
				});
		return getActiveOrdersResponse.getOrderIds();
	}

	private List<String> markActiveOrders() {
		List<Order> orders = getOrders();

		List<String> activeOrders = new ArrayList<String>();

		activeOrders.add(orders.get(0).getOrderId());

		ServiceClient<MarkActiveOrdersRequest, MarkActiveOrdersResponse> markActiveOrdersClient = null;
		try {
			markActiveOrdersClient = node.newServiceClient(
					"RosWebInterface/mark_active_orders", Order._TYPE);
		} catch (Exception e) {
		}
		MarkActiveOrdersRequest request = markActiveOrdersClient.newMessage();
		request.setOrderIds(activeOrders);
		markActiveOrdersClient.call(request,
				new ServiceResponseListener<MarkActiveOrdersResponse>() {

					@Override
					public void onSuccess(MarkActiveOrdersResponse arg0) {
						markActiveOrdersResponse = arg0;
					}

					@Override
					public void onFailure(RemoteException arg0) {
						log.error("Failed to mark active orders");
					}
				});
		return activeOrders;
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

	private MoveBaseActionGoal createGoalMessage(int id) {
		return createGoalMessage(id, false);
	}

	private MoveBaseActionGoal createGoalMessage(int id, boolean base) {
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
				log.error("Failed to find marker " + id);
			}
		} catch (Exception e) {
			e.printStackTrace();
		}
		return goalMsg;
	}
}
