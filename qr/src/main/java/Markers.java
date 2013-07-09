import java.io.File;
import java.io.IOException;
import java.io.StringReader;

import move_base_msgs.MoveBaseActionGoal;

import org.apache.commons.logging.Log;
import org.jdom2.Document;
import org.jdom2.JDOMException;
import org.jdom2.input.SAXBuilder;
import org.jdom2.input.sax.XMLReaders;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;
import org.xml.sax.EntityResolver;
import org.xml.sax.InputSource;
import org.xml.sax.SAXException;

import std_msgs.Int32;
import visualization_msgs.InteractiveMarkerInit;

public class Markers extends AbstractNodeMain {

	private InteractiveMarkerInit currentUpdate = null;
	private File locations = new File("locations.xml");

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_markers");
	}

	@Override
	public void onStart(ConnectedNode node) {
		final Log log = node.getLog();

		readXML();

		Subscriber<InteractiveMarkerInit> markerUpdateSub = node.newSubscriber("marker_server/update_full", InteractiveMarkerInit._TYPE);

		markerUpdateSub.addMessageListener(new MessageListener<InteractiveMarkerInit>() {
			@Override
			public void onNewMessage(InteractiveMarkerInit update) {
				currentUpdate = update;
			}
		});

		Subscriber<Int32> goalPointSub = node.newSubscriber("qr_markers/goal", Int32._TYPE);
		final Publisher<MoveBaseActionGoal> goalPub = node.newPublisher("move_base/goal", MoveBaseActionGoal._TYPE);

		goalPointSub.addMessageListener(new MessageListener<Int32>() {

			@Override
			public void onNewMessage(Int32 goalPoint) {
				MoveBaseActionGoal goalMsg = goalPub.newMessage();

				try {
					if (currentUpdate == null) {
						Thread.sleep(1000);
						if (currentUpdate == null) {
							log.error("Waited 1s, currentUpdate still null");
						}
					}
					goalMsg.getGoal().getTargetPose().getHeader().setFrameId("map");

					boolean valid = false;

					for (int i = 0; i < currentUpdate.getMarkers().size(); i++) {
						if (currentUpdate.getMarkers().get(i).getName().equals("point " + goalPoint.getData())) {
							valid = true;
							goalMsg.getGoal().getTargetPose().setPose(currentUpdate.getMarkers().get(i).getPose());
						}
					}

					if (valid) {
						goalPub.publish(goalMsg);
					} else {
						log.error("Failed to find marker " + goalPoint.getData());
					}
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		});

	}

	private void readXML() {
		if (locations.exists()) {
			Document doc = null;
			SAXBuilder builder = new SAXBuilder(XMLReaders.NONVALIDATING);
			builder.setEntityResolver(new EntityResolver() {
				@Override
				public InputSource resolveEntity(String publicId, String systemId) throws SAXException, IOException {
					return new InputSource(new StringReader(""));
				}
			});

			try {
				doc = builder.build(locations);
			} catch (JDOMException ex) {
				System.err.println(locations.getAbsolutePath() + " is either not a well-formed XML document or is not valid: "
						+ ex.getMessage());
			} catch (IOException ex) {
				ex.printStackTrace();
			}

			System.out.println("!!!!! " + doc.getRootElement().getChildren().size());
		} else {
			System.out.println("Locations file does not exist.");
		}
	}
}
