package butler.butler;

import geometry_msgs.Point;
import geometry_msgs.Pose;
import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Quaternion;
import geometry_msgs.Twist;

import java.util.ArrayList;

import move_base_msgs.MoveBaseActionGoal;
import nav_msgs.Path;

import org.ros.exception.RemoteException;
import org.ros.exception.ServiceNotFoundException;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import std_msgs.Bool;
import std_msgs.Empty;
import talker.Speach;
import talker.SpeachRequest;
import talker.SpeachResponse;
import web_connector.GetOrdersRequest;
import web_connector.GetOrdersResponse;
import web_connector.MarkOrderCompleteRequest;
import web_connector.MarkOrderCompleteResponse;
import web_connector.Order;
import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;

public class Queue extends AbstractNodeMain {

	private final int MAX_RECOVERY_LOOPS = 3;

	private GoalStatusArray lastStatus;
	private Publisher<GoalID> cancelPub;
	private Publisher<MoveBaseActionGoal> goalPub;
	private Publisher<PoseArray> butlerGoalPub;
	private Publisher<Bool> recoveryBehaviourPub, resultPub;
	private Publisher<PoseArray> recoveryPlanPub;
	private Publisher<std_msgs.String> feedbackPub;

	private ArrayList<QueueGoal> goals = new ArrayList<QueueGoal>();
	private MoveBaseActionGoal baseGoal;
	private static int goalNumber = 0;
	private boolean recoveryAttempted = false;
	private PoseWithCovarianceStamped currentLocation;
	private Path currentRecoveryPlan = null;
	private int recoveryNumber = 90;
	private int recoveryLoop = 1;
	private ConnectedNode node;
	private String validGoal = "";
	private String invalidGoal = "";
	private Object goalsLock = new Object();

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("butler/queue");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		this.node = node;
		Subscriber<PoseWithCovarianceStamped> locationSub = node.newSubscriber(
				"amcl_pose", PoseWithCovarianceStamped._TYPE);

		locationSub
				.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
					@Override
					public void onNewMessage(PoseWithCovarianceStamped location) {
						currentLocation = location;
					}
				});

		Subscriber<std_msgs.String> plannerSub = node.newSubscriber(
				"butler/planner/valid_goal", std_msgs.String._TYPE);

		plannerSub.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String update) {
				validGoal = update.getData();
			}
		});

		Subscriber<Path> recoveryPlanSub = node.newSubscriber(
				"butler_planner/planner_no_sensor/plan", Path._TYPE);

		recoveryPlanSub.addMessageListener(new MessageListener<Path>() {
			@Override
			public void onNewMessage(Path update) {
				currentRecoveryPlan = update;
			}
		});

		Subscriber<std_msgs.String> plannerInvalidSub = node.newSubscriber(
				"butler/planner/invalid_goal", std_msgs.String._TYPE);

		plannerInvalidSub
				.addMessageListener(new MessageListener<std_msgs.String>() {
					@Override
					public void onNewMessage(std_msgs.String update) {
						invalidGoal = update.getData();
					}
				});

		Subscriber<Twist> velSub = node.newSubscriber("b21/cmd_vel",
				Twist._TYPE);

		velSub.addMessageListener(new MessageListener<Twist>() {
			@Override
			public void onNewMessage(Twist update) {
				if (!zeroVelocity(update)) {
					recoveryNumber = 90;
					recoveryLoop = 1;
				}
			}
		});

		Subscriber<MoveBaseActionGoal> goalSub = node.newSubscriber(
				"butler/goal", MoveBaseActionGoal._TYPE);

		goalSub.addMessageListener(new MessageListener<MoveBaseActionGoal>() {
			@Override
			public void onNewMessage(MoveBaseActionGoal update) {
				addQRGoal(update);
			}
		});

		Subscriber<Bool> stopSub = node.newSubscriber("crowded_nav/stop",
				Bool._TYPE);

		stopSub.addMessageListener(new MessageListener<Bool>() {
			@Override
			public void onNewMessage(Bool update) {
				feedback("Stopping...");
				result(true);
				if (update.getData()) {
					synchronized (goalsLock) {
						while (goals.size() > 0) {
							try {
								goals.remove(goals.size() - 1);
							} catch (Exception e) {
								feedback("Failed to remove goal from Queue");
							}
						}
					}

					cancelAllGoals();
				}
			}
		});

		Subscriber<MoveBaseActionGoal> baseGoalSub = node.newSubscriber(
				"butler/base_goal", MoveBaseActionGoal._TYPE);

		baseGoalSub
				.addMessageListener(new MessageListener<MoveBaseActionGoal>() {
					@Override
					public void onNewMessage(MoveBaseActionGoal update) {
						baseGoal = update;
					}
				});

		Subscriber<GoalStatusArray> statusSub = node.newSubscriber(
				"move_base/status", GoalStatusArray._TYPE);

		statusSub.addMessageListener(new MessageListener<GoalStatusArray>() {
			@Override
			public void onNewMessage(GoalStatusArray update) {

				if (goals.size() > 0
						&& goals.get(0).getStatus() == QueueGoal.STOPPED_STATUS) {
					executeGoal();
				}

				if (goals.size() > 0
						&& update.getStatusList().size() > 0
						&& update.getStatusList().get(0).getStatus() == GoalStatus.SUCCEEDED) {
					recoveryNumber = 90;
					recoveryLoop = 1;
					if (goals
							.get(0)
							.getGoal()
							.getGoalId()
							.getId()
							.equals(update.getStatusList().get(0).getGoalId()
									.getId())) {
						System.out.println(goals.get(0).getGoal().getGoalId()
								.getId()
								+ " "
								+ update.getStatusList().get(0).getGoalId()
										.getId());

						if (goals.get(0).getType() != QueueGoal.RECOVERY_TYPE) {
							recoveryAttempted = false;
						}

						if (goals.get(0).getType() == QueueGoal.QR_TYPE) {
							result(true);
						}
						synchronized (goalsLock) {
							goals.remove(0);
						}
					} else {
						// TODO Deal with move_base...
						System.out.println("Waiting for move_base... "
								+ goals.get(0).getGoal().getGoalId().getId()
								+ " "
								+ update.getStatusList().get(0).getGoalId()
										.getId());
					}

				}

				if (goals.size() > 0
						&& update.getStatusList().size() > 0
						&& update.getStatusList().get(0).getStatus() == GoalStatus.ABORTED) {

					if (goals
							.get(0)
							.getGoal()
							.getGoalId()
							.getId()
							.equals(update.getStatusList().get(0).getGoalId()
									.getId())) {
						if (goals.get(0).getType() == QueueGoal.RECOVERY_TYPE) {
							synchronized (goalsLock) {
								goals.remove(0);
							}
						}

						if (recoveryNumber != 0) {
							executeRecovery();
						}
					}
				}

				lastStatus = update;
			}
		});

		goalPub = node.newPublisher("move_base/goal", MoveBaseActionGoal._TYPE);
		butlerGoalPub = node.newPublisher("butler/planner/goal",
				PoseArray._TYPE);
		cancelPub = node.newPublisher("move_base/cancel", GoalID._TYPE);
		recoveryBehaviourPub = node.newPublisher(
				"butler/enable_recovery_behaviours", Bool._TYPE);
		recoveryPlanPub = node.newPublisher("butler/planner/make_plan",
				PoseArray._TYPE);
		feedbackPub = node.newPublisher("crowded_nav/feedback",
				std_msgs.String._TYPE);
		resultPub = node.newPublisher("crowded_nav/result", Bool._TYPE);

		try {
			Thread.sleep(2000);
			cancelAllGoals();
			Thread.sleep(2000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}

	}

	private void cancelAllGoals() {
		for (GoalStatus s : lastStatus.getStatusList()) {
			GoalID newMsg = cancelPub.newMessage();
			newMsg.setId(s.getGoalId().getId());
			cancelPub.publish(newMsg);
		}
	}

	private void addQRGoal(MoveBaseActionGoal goal) {
		if (goals.size() == 0) {

			goal.getGoalId().setId(goalNumber + "");
			goalNumber++;
			QueueGoal newQueueGoal = new QueueGoal(goal, QueueGoal.QR_TYPE);
			synchronized (goalsLock) {
				goals.add(newQueueGoal);
			}

			System.out.println("New queue:");

		} else {
			feedback("Queue not empty. Discarding goal.");
		}

		for (QueueGoal qg : goals) {
			System.out.println(qg.getGoal().getGoalId().getId()
					+ ": ("
					+ qg.getGoal().getGoal().getTargetPose().getPose()
							.getPosition().getX()
					+ ","
					+ qg.getGoal().getGoal().getTargetPose().getPose()
							.getPosition().getY() + ")");
		}
		System.out.println();
	}

	private void executeGoal() {
		System.out.println("Publishing...");
		cancelAllGoals();
		if (goals.get(0).getType() == QueueGoal.RECOVERY_TYPE) {
			setRecoveryBehaviour(false);
		} else {
			setRecoveryBehaviour(true);
		}

		goalPub.publish(goals.get(0).getGoal());
		goals.get(0).setStatus(QueueGoal.RUNNING_STATUS);
	}

	private void executeRecovery() {

		if (recoveryNumber == 90) {
			makeRecoveryPlan();
			feedback("Attempting recovery loop "+recoveryLoop+"...");
		}

		System.out.println("Attempting recovery... " + recoveryNumber + "%");
		cancelAllGoals();
		goals.get(0).setStatus(QueueGoal.STOPPED_STATUS);
		MoveBaseActionGoal newGoalMsg = goalPub.newMessage();

		Path path = currentRecoveryPlan;
		System.out.println("Path length: " + path.getPoses().size());
		System.out.println("Trying pose: "
				+ (int) (path.getPoses().size() * 0.01 * recoveryNumber));

		newGoalMsg
				.getGoal()
				.setTargetPose(
						path.getPoses()
								.get((int) (path.getPoses().size() * 0.01 * recoveryNumber)));

		newGoalMsg.getGoal().getTargetPose().getHeader().setFrameId("map");

		newGoalMsg.getGoalId().setId(
				goals.get(0).getGoal().getGoalId().getId() + "_rec_"
						+ recoveryNumber);
		Quaternion q = calculateQuaternion(newGoalMsg.getGoal().getTargetPose()
				.getPose());
		newGoalMsg.getGoal().getTargetPose().getPose().setOrientation(q);

		PoseArray planMsg = butlerGoalPub.newMessage();
		planMsg.getPoses().add(currentLocation.getPose().getPose());
		planMsg.getPoses().add(newGoalMsg.getGoal().getTargetPose().getPose());
		planMsg.getHeader().setFrameId(newGoalMsg.getGoalId().getId());
		butlerGoalPub.publish(planMsg);

		boolean publish = false;
		validGoal = "";
		invalidGoal = "";

		for (int i = 0; i < 50; i++) {
			System.out.println(validGoal + ":" + invalidGoal);
			if (validGoal.equalsIgnoreCase(newGoalMsg.getGoalId().getId())) {
				publish = true;
				break;
			} else if (invalidGoal.equalsIgnoreCase(newGoalMsg.getGoalId()
					.getId())) {
				break;
			}

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		if (publish) {
			synchronized (goalsLock) {
				goals.add(0, new QueueGoal(newGoalMsg, QueueGoal.RECOVERY_TYPE));
				System.out.println("Publishing: "
						+ newGoalMsg.getGoalId().getId());
			}
		}

		System.out.println("New queue:");
		for (QueueGoal qg : goals) {
			Point p = qg.getGoal().getGoal().getTargetPose().getPose()
					.getPosition();
			System.out.println(p.getX() + " " + p.getY());
		}
		System.out.println();

		if (recoveryNumber == 90 && recoveryAttempted) {
			speak("Excuse me");
			try {
				Thread.sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		if (recoveryNumber > 10) {
			recoveryNumber -= 10;
		} else {
			recoveryNumber -= 2;

			if (recoveryNumber == 0) {
				if (recoveryLoop == MAX_RECOVERY_LOOPS) {
					feedback("Attempted " + MAX_RECOVERY_LOOPS
							+ " recovery loops. Aborting...");
					result(false);
					cancelAllGoals();
				} else {
					recoveryNumber = 90;
				}
			}
		}
		recoveryAttempted = true;
	}

	private void speak(String message) {
		ServiceClient<talker.SpeachRequest, talker.SpeachResponse> serviceClient;
		try {
			serviceClient = node.newServiceClient("say", Speach._TYPE);

			final SpeachRequest request = serviceClient.newMessage();
			std_msgs.String string = (std_msgs.String) node
					.getTopicMessageFactory().newFromType("std_msgs/String");
			string.setData(message);
			request.setText(string);
			serviceClient.call(request,
					new ServiceResponseListener<SpeachResponse>() {

						@Override
						public void onFailure(RemoteException arg0) {
						}

						@Override
						public void onSuccess(SpeachResponse arg0) {
						}
					});
		} catch (ServiceNotFoundException e) {
			e.printStackTrace();
		}
	}

	private boolean zeroVelocity(Twist twist) {
		return twist.getAngular().getX() == 0 && twist.getAngular().getY() == 0
				&& twist.getAngular().getZ() == 0
				&& twist.getLinear().getX() == 0
				&& twist.getLinear().getY() == 0
				&& twist.getLinear().getZ() == 0;
	}

	private void setRecoveryBehaviour(boolean value) {
		Bool recovery = recoveryBehaviourPub.newMessage();
		recovery.setData(value);
		recoveryBehaviourPub.publish(recovery);
	}

	private Quaternion calculateQuaternion() {
		try {
			return calculateQuaternion(goals.get(0).getGoal().getGoal()
					.getTargetPose().getPose());
		} catch (IndexOutOfBoundsException e) {
			System.out.println("No goal set");
			calculateQuaternion(baseGoal.getGoal().getTargetPose().getPose());
		}
		return goalPub.newMessage().getGoal().getTargetPose().getPose()
				.getOrientation();
	}

	private Quaternion calculateQuaternion(Pose pose) {
		double z = (currentLocation.getPose().getPose().getOrientation().getZ());
		double currentAngle = Math.toDegrees(2 * Math.asin(z));
		double rAngle = 360.0 - currentAngle;
		double x1 = currentLocation.getPose().getPose().getPosition().getX(), y1 = currentLocation
				.getPose().getPose().getPosition().getY(), x2 = pose
				.getPosition().getX(), y2 = pose.getPosition().getY();
		double xDiff = x2 - x1, yDiff = y2 - y1;

		double tAngle = Math.toDegrees(Math.atan2(yDiff, xDiff));

		double wGoal = Math.cos(Math.toRadians(tAngle) / 2);
		double zGoal = Math.sin(Math.toRadians(tAngle) / 2);

		Quaternion q = goalPub.newMessage().getGoal().getTargetPose().getPose()
				.getOrientation();

		q.setW(wGoal);
		q.setZ(zGoal);

		System.out.println("Quaternion: z:" + zGoal + ", w:" + wGoal);
		return q;
	}

	private boolean makeRecoveryPlan() {
		while (currentLocation == null) {
			System.out.println("Waiting for location...");
			try {
				Thread.sleep(500);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		currentRecoveryPlan = null;

		PoseArray recoveryPlanMsg = recoveryPlanPub.newMessage();
		recoveryPlanMsg.getPoses().add(currentLocation.getPose().getPose());
		recoveryPlanMsg.getPoses().add(
				goals.get(0).getGoal().getGoal().getTargetPose().getPose());
		recoveryPlanMsg.getHeader().setFrameId(
				goals.get(0).getGoal().getGoalId().getId() + "_recplan");
		recoveryPlanPub.publish(recoveryPlanMsg);

		boolean planSucceeded = false;

		for (int i = 0; i < 50; i++) {
			if (currentRecoveryPlan != null) {
				if (currentRecoveryPlan.getPoses().size() > 0) {
					planSucceeded = true;
					System.out.println("Received initial recovery plan");
				}
				break;
			} else {
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}

		if (!planSucceeded) {
			System.out.println("Failed to receive initial recovery plan");
		}
		return planSucceeded;
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