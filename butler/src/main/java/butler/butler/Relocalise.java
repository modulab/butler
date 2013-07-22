package butler.butler;

import geometry_msgs.PoseWithCovarianceStamped;

import java.awt.geom.AffineTransform;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Empty;

public class Relocalise extends AbstractNodeMain {

	private PoseWithCovarianceStamped currentLocation = null;
	// Lab
	// private Point2D.Double p1 = new Point2D.Double(-2.644, 2.413), p2 = new
	// Point2D.Double(-0.169, -2.962), p3 = new Point2D.Double(14.781,
	// 10.260), p4 = new Point2D.Double(17.259, 5.105), p1b = null;

	// LG
	private Point2D.Double p1 = new Point2D.Double(-1.149, 5.108), p2 = new Point2D.Double(-13.040, -6.069), p3 = new Point2D.Double(
			15.486, -13.611), p4 = new Point2D.Double(2.832, -24.263), p1b = null;
	private Publisher<PoseWithCovarianceStamped> initialPosePub;
	private double mapRotation = 0;
	private Rectangle2D validArea = null;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("butler/relocalise");
	}

	@Override
	public void onStart(ConnectedNode node) {
		calculateMapRotation();
		Subscriber<PoseWithCovarianceStamped> locationSub = node.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);
		locationSub.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(PoseWithCovarianceStamped location) {
				if (isValidLocation(location)) {
					currentLocation = location;
				} else {
					relocalise();
				}
			}
		});

		Subscriber<Empty> relocaliseSub = node.newSubscriber("butler/relocalise", Empty._TYPE);
		relocaliseSub.addMessageListener(new MessageListener<Empty>() {
			@Override
			public void onNewMessage(Empty update) {
				relocalise();
			}
		});

		initialPosePub = node.newPublisher("initialpose", PoseWithCovarianceStamped._TYPE);
	}

	private boolean isValidLocation(PoseWithCovarianceStamped location) {
		AffineTransform at = AffineTransform.getRotateInstance(Math.toRadians(mapRotation), -37.15, -42.199999999999996);

		System.out.println(location.getPose().getPose().getPosition().getX() + " " + location.getPose().getPose().getPosition().getY());

		return validArea.contains(at.transform(new Point2D.Double(location.getPose().getPose().getPosition().getX(), location.getPose()
				.getPose().getPosition().getY()), p1b));
	}

	private void relocalise() {
		if (currentLocation != null) {
			System.out.println("Relocalising...");
			initialPosePub.publish(currentLocation);
		} else {
			System.out.println("Invalid position but the robot has not previously been localised");
		}
	}

	private void calculateMapRotation() {
		double lowestHeight = 100.0, lowestWidth = 100.0, lowestHeightAngle = -1, lowestWidthAngle = -1;

		for (double i = 45.0; i < 90.0; i += 0.1) {
			AffineTransform at = AffineTransform.getRotateInstance(Math.toRadians(i), -37.15, -42.199999999999996);
			validArea = new Rectangle2D.Double(at.transform(p1, p1b).getX(), at.transform(p1, p1b).getY(), 0, 0);
			validArea.add(at.transform(p1, p1b));
			validArea.add(at.transform(p2, p1b));
			validArea.add(at.transform(p3, p1b));
			validArea.add(at.transform(p4, p1b));

			if (validArea.getHeight() < lowestHeight) {
				lowestHeight = validArea.getHeight();
				lowestHeightAngle = i;
			}

			if (validArea.getWidth() < lowestWidth) {
				lowestWidth = validArea.getWidth();
				lowestWidthAngle = i;
			}
		}

		mapRotation = (lowestHeightAngle + lowestWidthAngle) / 2;
		System.out.println("Rotation " + mapRotation);

		AffineTransform at = AffineTransform.getRotateInstance(Math.toRadians(mapRotation), -37.15, -42.199999999999996);
		validArea = new Rectangle2D.Double(at.transform(p1, p1b).getX(), at.transform(p1, p1b).getY(), 0, 0);
		validArea.add(at.transform(p1, p1b));
		validArea.add(at.transform(p2, p1b));
		validArea.add(at.transform(p3, p1b));
		validArea.add(at.transform(p4, p1b));
	}
}
