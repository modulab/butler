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
import sensor_msgs.PointCloud2;
import std_msgs.Bool;
import actionlib_msgs.GoalStatusArray;

public class Brake extends AbstractNodeMain {

	private long lastLaser, lastPTU, lastKinect, lastMoveBase, lastOdom, lastVel, brakesLastApplied;
	private final long LASER_FAILURE_THRESHOLD = 500000000, PTU_FAILURE_THRESHOLD = 500000000, KINECT_FAILURE_THRESHOLD = 1000000000,
			MOVE_BASE_FAILURE_THRESHOLD = 500000000, VEL_FAILURE_THRESHOLD = 1500000000, ODOM_FAILURE_THRESHOLD = 500000000,
			BRAKE_THRESHOLD = 5000000000l;
	private boolean braked;
	private Publisher<Bool> brakePub;
	private Publisher<Twist> velPub;
	private Odometry lastOdomMsg = null;
	private Log log;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("brake");
	}

	@Override
	public void onStart(ConnectedNode node) {
		log = node.getLog();

		brakePub = node.newPublisher("b21/cmd_brake_power", Bool._TYPE);

		velPub = node.newPublisher("b21/cmd_vel", Twist._TYPE);

		Subscriber<LaserScan> laserSub = node.newSubscriber("scan", LaserScan._TYPE);

		laserSub.addMessageListener(new MessageListener<LaserScan>() {
			@Override
			public void onNewMessage(LaserScan update) {
				lastLaser = System.nanoTime();
			}
		});

		Subscriber<JointState> ptuSub = node.newSubscriber("ptu/state", JointState._TYPE);

		ptuSub.addMessageListener(new MessageListener<JointState>() {
			@Override
			public void onNewMessage(JointState update) {
				lastPTU = System.nanoTime();
			}
		});

		Subscriber<PointCloud2> kinectSub = node.newSubscriber("voxel_grid/output", PointCloud2._TYPE);

		kinectSub.addMessageListener(new MessageListener<PointCloud2>() {
			@Override
			public void onNewMessage(PointCloud2 update) {
				lastKinect = System.nanoTime();
			}
		});

		Subscriber<GoalStatusArray> moveBaseSub = node.newSubscriber("move_base/status", GoalStatusArray._TYPE);

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
				lastOdom = System.nanoTime();
				lastOdomMsg = update;
			}
		});

		Subscriber<Twist> velSub = node.newSubscriber("b21/cmd_vel", Twist._TYPE);

		velSub.addMessageListener(new MessageListener<Twist>() {
			@Override
			public void onNewMessage(Twist update) {
				lastVel = System.nanoTime();
			}
		});

		Subscriber<Bool> brakeSub = node.newSubscriber("b21/brake_power", Bool._TYPE);

		brakeSub.addMessageListener(new MessageListener<Bool>() {
			@Override
			public void onNewMessage(Bool update) {
				// if (!update.getData())
				// braked = update.getData();
				// System.out.println(braked);
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

					System.out.println("Laser messages not received for over 500ms. Braking.");
					brake();
				}

				else if (System.nanoTime() - lastPTU > PTU_FAILURE_THRESHOLD) {

					System.out.println("PTU messages not received for over 500ms. Braking.");
					brake();
				}

				else if (System.nanoTime() - lastKinect > KINECT_FAILURE_THRESHOLD) {

					System.out.println("Kinect messages not received for over 1000ms. Braking.");
					brake();
				}

				else if (System.nanoTime() - lastMoveBase > MOVE_BASE_FAILURE_THRESHOLD) {

					System.out.println("Move_base messages not received for over 500ms. Braking.");
					brake();
					velPub.publish(velPub.newMessage());
				}

				else if (System.nanoTime() - lastVel > VEL_FAILURE_THRESHOLD) {
					if (lastOdomMsg == null) {
						System.out.println("No odometry messages received. Braking.");
						brake();
					}

					// else if ((System.nanoTime() - lastOdom <
					// ODOM_FAILURE_THRESHOLD)
					// && (lastOdomMsg.getTwist().getTwist().getLinear().getX()
					// > 0.04
					// || lastOdomMsg.getTwist().getTwist().getLinear().getX() <
					// -0.04
					// || lastOdomMsg.getTwist().getTwist().getLinear().getY() >
					// 0.04
					// || lastOdomMsg.getTwist().getTwist().getLinear().getY() <
					// -0.04
					// || lastOdomMsg.getTwist().getTwist().getAngular().getZ()
					// > 0.09 || lastOdomMsg.getTwist().getTwist()
					// .getAngular().getZ() < -0.09)) {
					// System.out.println("No velocity messages received for over 500ms but the robot is still moving. Braking.");
					// brake();
					// velPub.publish(velPub.newMessage());
					// }
				}
			}
			try {
				Thread.sleep(50);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private void brake() {
		Bool newBrakeMsg = brakePub.newMessage();
		newBrakeMsg.setData(true);
		brakePub.publish(newBrakeMsg);
		brakesLastApplied = System.nanoTime();
		braked = true;
	}
}
