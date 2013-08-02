package butler.butler;

import geometry_msgs.PoseArray;
import geometry_msgs.PoseWithCovarianceStamped;
import geometry_msgs.Twist;

import java.util.ArrayList;

import move_base_msgs.MoveBaseActionGoal;

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

import std_msgs.Empty;
import talker.Speach;
import talker.SpeachRequest;
import talker.SpeachResponse;
import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;

public class Queue extends AbstractNodeMain {

	private GoalStatusArray lastStatus;
	private Publisher<GoalID> cancelPub;
	private Publisher<MoveBaseActionGoal> goalPub;
	private Publisher<PoseArray> butlerGoalPub;
	private ArrayList<QueueGoal> goals = new ArrayList<QueueGoal>();
	private MoveBaseActionGoal baseGoal;
	private static int goalNumber = 0;
	private boolean waitForDrinks = false, recoveryAttempted = false;
	private PoseWithCovarianceStamped currentLocation;
	private int recoveryNumber = 90;
	private ConnectedNode node;
	private final boolean GO_TO_BASE = false;
	private String validGoal = "";
	private String invalidGoal = "";

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("butler/queue");
	}

	@Override
	public void onStart(final ConnectedNode node) {
		this.node = node;
		Subscriber<PoseWithCovarianceStamped> locationSub = node.newSubscriber("amcl_pose", PoseWithCovarianceStamped._TYPE);

		locationSub.addMessageListener(new MessageListener<PoseWithCovarianceStamped>() {
			@Override
			public void onNewMessage(PoseWithCovarianceStamped location) {
				currentLocation = location;
			}
		});

		Subscriber<std_msgs.String> plannerSub = node.newSubscriber("butler/planner/valid_goal", std_msgs.String._TYPE);

		plannerSub.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String update) {
				validGoal = update.getData();
			}
		});

		Subscriber<std_msgs.String> plannerInvalidSub = node.newSubscriber("butler/planner/invalid_goal", std_msgs.String._TYPE);

		plannerInvalidSub.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String update) {
				invalidGoal = update.getData();
			}
		});

		Subscriber<Twist> velSub = node.newSubscriber("b21/cmd_vel", Twist._TYPE);

		velSub.addMessageListener(new MessageListener<Twist>() {
			@Override
			public void onNewMessage(Twist update) {
				if (!zeroVelocity(update)) {
					recoveryNumber = 90;
				}
			}
		});

		Subscriber<MoveBaseActionGoal> goalSub = node.newSubscriber("butler/goal", MoveBaseActionGoal._TYPE);

		goalSub.addMessageListener(new MessageListener<MoveBaseActionGoal>() {
			@Override
			public void onNewMessage(MoveBaseActionGoal update) {
				addQRGoal(update);
			}
		});

		Subscriber<Empty> drinksAddedSub = node.newSubscriber("butler/drinks_added", Empty._TYPE);

		drinksAddedSub.addMessageListener(new MessageListener<Empty>() {
			@Override
			public void onNewMessage(Empty update) {
				waitForDrinks = false;
			}
		});

		Subscriber<MoveBaseActionGoal> baseGoalSub = node.newSubscriber("butler/base_goal", MoveBaseActionGoal._TYPE);

		baseGoalSub.addMessageListener(new MessageListener<MoveBaseActionGoal>() {
			@Override
			public void onNewMessage(MoveBaseActionGoal update) {
				baseGoal = update;
			}
		});

		Subscriber<GoalStatusArray> statusSub = node.newSubscriber("move_base/status", GoalStatusArray._TYPE);

		statusSub.addMessageListener(new MessageListener<GoalStatusArray>() {
			@Override
			public void onNewMessage(GoalStatusArray update) {
				if (goals.size() > 0 && goals.get(0).getStatus() == QueueGoal.STOPPED_STATUS) {
					executeGoal();
				}

				if (goals.size() > 0 && update.getStatusList().size() > 0
						&& update.getStatusList().get(0).getStatus() == GoalStatus.SUCCEEDED) {
					recoveryNumber = 90;
					System.out.println("c");
					if (goals.get(0).getGoal().getGoalId().getId().equals(update.getStatusList().get(0).getGoalId().getId())) {
						System.out.println(goals.get(0).getGoal().getGoalId().getId() + " "
								+ update.getStatusList().get(0).getGoalId().getId());
						System.out.println("a");

						if (goals.get(0).getType() != QueueGoal.RECOVERY_TYPE) {
							recoveryAttempted = false;
						}

						if (goals.get(0).getType() == QueueGoal.BASE_TYPE) {
							System.out.println("b");
							waitForDrinks = true;
							try {
								while (waitForDrinks) {
									Thread.sleep(1000);
									System.out.println("Waiting for drinks...");
								}
								Thread.sleep(3000);
							} catch (InterruptedException e) {
								e.printStackTrace();
							}
						}

						else if (goals.get(0).getType() == QueueGoal.RECOVERY_TYPE) {
							// speak("Excuse me", node);
						}

						goals.remove(0);
					} else {
						System.out.println("Waiting for move_base... " + goals.get(0).getGoal().getGoalId().getId() + " "
								+ update.getStatusList().get(0).getGoalId().getId());
					}

				}

				if (goals.size() > 0 && update.getStatusList().size() > 0
						&& update.getStatusList().get(0).getStatus() == GoalStatus.ABORTED) {

					if (goals.get(0).getGoal().getGoalId().getId().equals(update.getStatusList().get(0).getGoalId().getId())) {
						if (goals.get(0).getType() == QueueGoal.RECOVERY_TYPE) {
							goals.remove(0);
						}

						if (recoveryNumber != 0) {
							executeRecovery2();
						}
					}
				}

				lastStatus = update;
			}
		});

		goalPub = node.newPublisher("move_base/goal", MoveBaseActionGoal._TYPE);
		butlerGoalPub = node.newPublisher("butler/planner/goal", PoseArray._TYPE);
		cancelPub = node.newPublisher("move_base/cancel", GoalID._TYPE);

		try {
			Thread.sleep(2000);
			cancelAllGoals();
			Thread.sleep(2000);

			// addRandomGoal();
			// addQRGoal(goalPub.newMessage());
			// testGoal();
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

	private void testGoal() {
		MoveBaseActionGoal testMsg = goalPub.newMessage();
		testMsg.getGoalId().setId("99");
		testMsg.getGoal().getTargetPose().getPose().getPosition().setX(1.7322527721303649);
		testMsg.getGoal().getTargetPose().getPose().getOrientation().setW(0.9498120473522184);
		goalPub.publish(testMsg);
	}

	private void addQRGoal(MoveBaseActionGoal newGoal) {
		int i;
		for (i = 0; i < goals.size(); i++) {
			if (goals.get(i).getType() != QueueGoal.QR_TYPE) {
				break;
			}
		}

		newGoal.getGoalId().setId(goalNumber + "");
		goalNumber++;
		QueueGoal newQueueGoal = new QueueGoal(newGoal, QueueGoal.QR_TYPE);

		if (i > 0) {
			goals.get(i - 1);
			if (!goals.get(i - 1).equals(newQueueGoal)) {
				if (GO_TO_BASE) {
					addBaseGoal(i);
					goals.add(i + 1, newQueueGoal);
				} else {
					goals.add(i, newQueueGoal);
				}
			}
		} else {
			if (GO_TO_BASE) {
				addBaseGoal(i);
				goals.add(i + 1, newQueueGoal);
			} else {
				goals.add(i, newQueueGoal);
			}
		}

		System.out.println("New queue:");
		for (QueueGoal qg : goals) {
			System.out.println(qg.getGoal().getGoal().getTargetPose().getPose().getPosition().getX());
		}
		System.out.println();
	}

	private void addRandomGoal() {
		QueueGoal newQueueGoal = new QueueGoal(goalPub.newMessage(), QueueGoal.RANDOM_TYPE);
		newQueueGoal.getGoal().getGoal().getTargetPose().getPose().getPosition().setX(2.222);

		newQueueGoal.getGoal().getGoal().getTargetPose().getPose().getOrientation().setW(0.9498120473522184);
		goals.add(newQueueGoal);
	}

	private void executeGoal() {
		System.out.println("pub");
		cancelAllGoals();
		goalPub.publish(goals.get(0).getGoal());

		// PoseArray planMsg = butlerGoalPub.newMessage();
		// planMsg.getPoses().add(currentLocation.getPose().getPose());
		// planMsg.getPoses().add(
		// goals.get(0).getGoal().getGoal().getTargetPose().getPose());
		// planMsg.getHeader().setFrameId(
		// goals.get(0).getGoal().getGoalId().getId());
		// butlerGoalPub.publish(planMsg);

		goals.get(0).setStatus(QueueGoal.RUNNING_STATUS);
	}

	private void addBaseGoal(int i) {
		if (baseGoal != null) {
			MoveBaseActionGoal newBaseGoal = goalPub.newMessage();
			newBaseGoal.getGoalId().setId(goalNumber + "");
			newBaseGoal.getGoal().getTargetPose().getHeader().setFrameId("map");
			newBaseGoal.getGoal().getTargetPose().getPose().getPosition()
					.setX(baseGoal.getGoal().getTargetPose().getPose().getPosition().getX());
			newBaseGoal.getGoal().getTargetPose().getPose().getPosition()
					.setY(baseGoal.getGoal().getTargetPose().getPose().getPosition().getY());
			newBaseGoal.getGoal().getTargetPose().getPose().getOrientation()
					.setZ(baseGoal.getGoal().getTargetPose().getPose().getOrientation().getZ());
			newBaseGoal.getGoal().getTargetPose().getPose().getOrientation()
					.setW(baseGoal.getGoal().getTargetPose().getPose().getOrientation().getW());
			goalNumber++;
			goals.add(i, new QueueGoal(newBaseGoal, QueueGoal.BASE_TYPE));
		} else {
			System.out.println("Base goal has not been received");
		}
	}

	private void executeRecovery2() {
		System.out.println("Attempting recovery... " + recoveryNumber + "%");
		cancelAllGoals();
		goals.get(0).setStatus(QueueGoal.STOPPED_STATUS);
		MoveBaseActionGoal newGoalMsg = goalPub.newMessage();
		newGoalMsg
				.getGoal()
				.getTargetPose()
				.getPose()
				.getPosition()
				.setX(currentLocation.getPose().getPose().getPosition().getX()
						+ ((goals.get(0).getGoal().getGoal().getTargetPose().getPose().getPosition().getX() - currentLocation.getPose()
								.getPose().getPosition().getX()) * 0.01 * recoveryNumber));
		newGoalMsg
				.getGoal()
				.getTargetPose()
				.getPose()
				.getPosition()
				.setY(currentLocation.getPose().getPose().getPosition().getY()
						+ ((goals.get(0).getGoal().getGoal().getTargetPose().getPose().getPosition().getY() - currentLocation.getPose()
								.getPose().getPosition().getY()) * 0.01 * recoveryNumber));
		newGoalMsg.getGoal().getTargetPose().getPose().getOrientation().setW(1.0);
		newGoalMsg.getGoal().getTargetPose().getHeader().setFrameId("map");
		newGoalMsg.getGoalId().setId(goals.get(0).getGoal().getGoalId().getId() + "_rec_" + recoveryNumber);

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
			} else if (invalidGoal.equalsIgnoreCase(newGoalMsg.getGoalId().getId())) {
				break;
			}

			try {
				Thread.sleep(100);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}

		if (publish) {
			goals.add(0, new QueueGoal(newGoalMsg, QueueGoal.RECOVERY_TYPE));
			System.out.println("Publishing: " + newGoalMsg.getGoalId().getId());
		}

		System.out.println("New queue:");
		for (QueueGoal qg : goals) {
			System.out.println(qg.getGoal().getGoal().getTargetPose().getPose().getPosition().getX() + " "
					+ qg.getGoal().getGoal().getTargetPose().getPose().getPosition().getY());
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
				if (publish) {
					goals.remove(1);
				} else {
					goals.remove(0);
				}
				System.out.println("Failed to reach goal");
			}
		}
		recoveryAttempted = true;
	}

	private void executeRecovery() {
		System.out.println("Attempting recovery... " + recoveryNumber + "%");
		cancelAllGoals();
		goals.get(0).setStatus(QueueGoal.STOPPED_STATUS);
		MoveBaseActionGoal newGoalMsg = goalPub.newMessage();
		newGoalMsg
				.getGoal()
				.getTargetPose()
				.getPose()
				.getPosition()
				.setX(currentLocation.getPose().getPose().getPosition().getX()
						+ ((goals.get(0).getGoal().getGoal().getTargetPose().getPose().getPosition().getX() - currentLocation.getPose()
								.getPose().getPosition().getX()) * 0.01 * recoveryNumber));
		newGoalMsg
				.getGoal()
				.getTargetPose()
				.getPose()
				.getPosition()
				.setY(currentLocation.getPose().getPose().getPosition().getY()
						+ ((goals.get(0).getGoal().getGoal().getTargetPose().getPose().getPosition().getY() - currentLocation.getPose()
								.getPose().getPosition().getY()) * 0.01 * recoveryNumber));
		newGoalMsg.getGoal().getTargetPose().getPose().getOrientation().setW(1.0);
		newGoalMsg.getGoal().getTargetPose().getHeader().setFrameId("map");
		newGoalMsg.getGoalId().setId(goals.get(0).getGoal().getGoalId().getId() + "_rec_" + recoveryNumber);
		goals.add(0, new QueueGoal(newGoalMsg, QueueGoal.RECOVERY_TYPE));

		System.out.println("New queue:");
		for (QueueGoal qg : goals) {
			System.out.println(qg.getGoal().getGoal().getTargetPose().getPose().getPosition().getX() + " "
					+ qg.getGoal().getGoal().getTargetPose().getPose().getPosition().getY());
		}
		System.out.println();

		if (recoveryNumber == 90 && recoveryAttempted) {
			speak("Excuse me");
		}

		if (recoveryNumber > 10) {
			recoveryNumber -= 10;
		} else {
			recoveryNumber -= 2;

			if (recoveryNumber == 0) {
				goals.remove(1);
				System.out.println("Failed to reach goal");
			}
		}
		recoveryAttempted = true;
	}

	private void speak(String message) {
		ServiceClient<talker.SpeachRequest, talker.SpeachResponse> serviceClient;
		try {
			serviceClient = node.newServiceClient("say", Speach._TYPE);

			final SpeachRequest request = serviceClient.newMessage();
			std_msgs.String string = (std_msgs.String) node.getTopicMessageFactory().newFromType("std_msgs/String");
			string.setData(message);
			request.setText(string);
			serviceClient.call(request, new ServiceResponseListener<SpeachResponse>() {

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
		return twist.getAngular().getX() == 0 && twist.getAngular().getY() == 0 && twist.getAngular().getZ() == 0
				&& twist.getLinear().getX() == 0 && twist.getLinear().getY() == 0 && twist.getLinear().getZ() == 0;
	}
}
