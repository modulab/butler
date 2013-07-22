package butler.butler;

import java.util.ArrayList;

import move_base_msgs.MoveBaseActionGoal;

import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

import actionlib_msgs.GoalID;
import actionlib_msgs.GoalStatus;
import actionlib_msgs.GoalStatusArray;

public class Queue extends AbstractNodeMain {

	private GoalStatusArray lastStatus;
	private Publisher<GoalID> cancelPub;
	private Publisher<MoveBaseActionGoal> goalPub;
	private ArrayList<QueueGoal> goals = new ArrayList<QueueGoal>();
	private MoveBaseActionGoal baseGoal;
	private static int goalNumber = 0;

	@Override
	public GraphName getDefaultNodeName() {
		return GraphName.of("butler/queue");
	}

	@Override
	public void onStart(ConnectedNode node) {

		Subscriber<MoveBaseActionGoal> goalSub = node.newSubscriber("butler/goal", MoveBaseActionGoal._TYPE);

		goalSub.addMessageListener(new MessageListener<MoveBaseActionGoal>() {
			@Override
			public void onNewMessage(MoveBaseActionGoal update) {
				addQRGoal(update);
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
					if (goals.get(0).getGoal().getGoalId().getId().equals(update.getStatusList().get(0).getGoalId().getId())) {
						System.out.println(goals.get(0).getGoal().getGoalId().getId() + " "
								+ update.getStatusList().get(0).getGoalId().getId());
						goals.remove(0);
					} else {
						System.out.println("Waiting for move_base... " + goals.get(0).getGoal().getGoalId().getId() + " "
								+ update.getStatusList().get(0).getGoalId().getId());
					}

				}

				if (goals.size() > 0 && update.getStatusList().size() > 0
						&& update.getStatusList().get(0).getStatus() == GoalStatus.ABORTED) {

				}

				lastStatus = update;
			}
		});

		goalPub = node.newPublisher("move_base/goal", MoveBaseActionGoal._TYPE);
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
				addBaseGoal(i);
				goals.add(i + 1, newQueueGoal);
			}
		} else {
			addBaseGoal(i);
			goals.add(i + 1, newQueueGoal);
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
		cancelAllGoals();
		goalPub.publish(goals.get(0).getGoal());
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
}
