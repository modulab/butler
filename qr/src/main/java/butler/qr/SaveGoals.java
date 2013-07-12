package butler.qr;

import geometry_msgs.PoseWithCovarianceStamped;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.Scanner;

import org.jdom2.Document;
import org.jdom2.Element;
import org.jdom2.output.Format;
import org.jdom2.output.XMLOutputter;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

public class SaveGoals extends AbstractNodeMain {

	private int num;
	private PoseWithCovarianceStamped currentLocation = null;
	private Document doc = null;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("qr_save_goals");
	}

	@Override
	public void onStart(ConnectedNode node) {
		Scanner sc = new Scanner(System.in);

		Subscriber<PoseWithCovarianceStamped> locationSub = node.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
		locationSub.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(PoseWithCovarianceStamped location) {
				currentLocation = location;
			}
		});

		try {
			Thread.sleep(500);
			System.out.println("How many points?");
			num = Integer.parseInt(new BufferedReader(new InputStreamReader(System.in)).readLine());
		} catch (IOException e) {
			e.printStackTrace();
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

		Element locationsEl = new Element("locations");
		doc = new Document(locationsEl);

		for (int i = 0; i < num; i++) {
			System.out.println("Point: " + i);
			sc.nextLine();

			Element locationEl = new Element("location");
			locationsEl.addContent(locationEl);

			Element px = new Element("px");
			Element py = new Element("py");
			Element pz = new Element("pz");
			Element ox = new Element("ox");
			Element oy = new Element("oy");
			Element oz = new Element("oz");
			Element ow = new Element("ow");

			locationEl.addContent(px);
			locationEl.addContent(py);
			locationEl.addContent(pz);
			locationEl.addContent(ox);
			locationEl.addContent(oy);
			locationEl.addContent(oz);
			locationEl.addContent(ow);

			px.setText(currentLocation.getPose().getPose().getPosition().getX() + "");
			py.setText(currentLocation.getPose().getPose().getPosition().getY() + "");
			pz.setText(currentLocation.getPose().getPose().getPosition().getZ() + "");
			ox.setText(currentLocation.getPose().getPose().getOrientation().getX() + "");
			oy.setText(currentLocation.getPose().getPose().getOrientation().getY() + "");
			oz.setText(currentLocation.getPose().getPose().getOrientation().getZ() + "");
			ow.setText(currentLocation.getPose().getPose().getOrientation().getW() + "");
		}

		try {
			XMLOutputter xmlOut = new XMLOutputter(Format.getPrettyFormat());
			xmlOut.output(doc, new FileOutputStream(new File("/home/sean/ROS/butler_workspace/butler/qr/locations.xml")));
		} catch (Exception e) {
			e.printStackTrace();
		}

		System.out.println("Done.");

	}
}
