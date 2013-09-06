package butler.brake;

import geometry_msgs.Twist;
import nav_msgs.Odometry;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import sensor_msgs.JointState;
import sensor_msgs.LaserScan;
import sensor_msgs.PointCloud;
import sensor_msgs.PointCloud2;
import std_msgs.Bool;
import actionlib_msgs.GoalStatusArray;

public class Brake extends AbstractNodeMain {

	private long lastLaser, lastKinect, lastMoveBase, lastVel,
			brakesLastApplied, lastXtion;
	private final long LASER_FAILURE_THRESHOLD = 500000000,
			KINECT_FAILURE_THRESHOLD = 1000000000,
			XTION_FAILURE_THRESHOLD = 1000000000,
			MOVE_BASE_FAILURE_THRESHOLD = 500000000,
			VEL_FAILURE_THRESHOLD = 1500000000, BRAKE_THRESHOLD = 5000000000l;
	private Publisher<Bool> brakePub;
	private Publisher<Twist> velPub;
	private Publisher<std_msgs.String> feedbackPub;
	private Odometry lastOdomMsg = null;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("brake");
	}

	@Override
	public void onStart(ConnectedNode node) {
		brakePub = node.newPublisher("b21/cmd_brake_power", Bool._TYPE);
		velPub = node.newPublisher("b21/cmd_vel", Twist._TYPE);
		feedbackPub = node.newPublisher("butler_status_messages",
				std_msgs.String._TYPE);

		Subscriber<LaserScan> laserSub = node.newSubscriber("scan",
				LaserScan._TYPE);

		laserSub.addMessageListener(new MessageListener<LaserScan>() {
			@Override
			public void onNewMessage(LaserScan update) {
				lastLaser = System.nanoTime();
			}
		});

		Subscriber<PointCloud2> kinectSub = node.newSubscriber(
				"voxel_grid_kinect/output", PointCloud2._TYPE);

		kinectSub.addMessageListener(new MessageListener<PointCloud2>() {
			@Override
			public void onNewMessage(PointCloud2 update) {
				lastKinect = System.nanoTime();
			}
		});

		Subscriber<PointCloud2> xtionSub = node.newSubscriber(
				"voxel_grid_xtion/output", PointCloud2._TYPE);

		xtionSub.addMessageListener(new MessageListener<PointCloud2>() {
			@Override
			public void onNewMessage(PointCloud2 update) {
				lastXtion = System.nanoTime();
			}
		});

		Subscriber<GoalStatusArray> moveBaseSub = node.newSubscriber(
				"move_base/status", GoalStatusArray._TYPE);

		moveBaseSub.addMessageListener(new MessageListener<GoalStatusArray>() {
			@Override
			public void onNewMessage(GoalStatusArray update) {
				lastMoveBase = System.nanoTime();
			}
		});
		
		Subscriber<Odometry> odomSub = node.newSubscriber("b21/odom", Odometry._TYPE);

		odomSub.addMessageListener(new MessageListener<Odometry>() {
			@Override
			public void onNewMessage(Odometry update) {
				lastOdomMsg = update;
			}
		});

		Subscriber<Twist> velSub = node.newSubscriber("b21/cmd_vel",
				Twist._TYPE);

		velSub.addMessageListener(new MessageListener<Twist>() {
			@Override
			public void onNewMessage(Twist update) {
				lastVel = System.nanoTime();
			}
		});

		try {
			Thread.sleep(5000);
		} catch (InterruptedException e1) {
			e1.printStackTrace();
		}

		while (true) {
			if (System.nanoTime() - brakesLastApplied > BRAKE_THRESHOLD) {
				if (System.nanoTime() - lastLaser > LASER_FAILURE_THRESHOLD) {
					brake("Laser messages not received for over 500ms. Braking.");
				}

				else if (System.nanoTime() - lastKinect > KINECT_FAILURE_THRESHOLD) {
					brake("Kinect messages not received for over 1000ms. Braking.");
				}

				else if (System.nanoTime() - lastXtion > XTION_FAILURE_THRESHOLD) {
					brake("Xtion messages not received for over 1000ms. Braking.");
				}

				else if (System.nanoTime() - lastMoveBase > MOVE_BASE_FAILURE_THRESHOLD) {
					brake("Move_base messages not received for over 500ms. Braking.");
					velPub.publish(velPub.newMessage());
				}

				else if (System.nanoTime() - lastVel > VEL_FAILURE_THRESHOLD) {
					if (lastOdomMsg == null) {
						brake("No odometry messages received. Braking.");
					}
				}
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private void brake(String message) {
		feedback(message);
		Bool newBrakeMsg = brakePub.newMessage();
		newBrakeMsg.setData(true);
		brakePub.publish(newBrakeMsg);
		brakesLastApplied = System.nanoTime();
	}

	private void feedback(String message) {
		System.out.println(message);

		std_msgs.String feedbackMsg = feedbackPub.newMessage();
		feedbackMsg.setData(message);
		feedbackPub.publish(feedbackMsg);
	}
}
