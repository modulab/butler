package butler.butler;

import move_base_msgs.MoveBaseActionGoal;

public class QueueGoal {

	public final static int QR_TYPE = 0;
	public final static int RANDOM_TYPE = 1;
	public final static int BASE_TYPE = 2;
	public final static int RECOVERY_TYPE = 3;

	public final static int STOPPED_STATUS = 0;
	public final static int RUNNING_STATUS = 1;

	private MoveBaseActionGoal goal;
	private int type, status = STOPPED_STATUS;

	public QueueGoal(MoveBaseActionGoal goal, int type) {
		super();
		this.goal = goal;
		this.type = type;
	}

	public int getStatus() {
		return status;
	}

	public void setStatus(int status) {
		this.status = status;
	}

	public MoveBaseActionGoal getGoal() {
		return goal;
	}

	public void setGoal(MoveBaseActionGoal goal) {
		this.goal = goal;
	}

	public int getType() {
		return type;
	}

	public void setType(int type) {
		this.type = type;
	}

	public boolean equals(QueueGoal other) {
		return this.getGoal().getGoal().getTargetPose().getPose().getPosition()
				.getX() == other.getGoal().getGoal().getTargetPose().getPose()
				.getPosition().getX()
				&& this.getGoal().getGoal().getTargetPose().getPose()
						.getPosition().getY() == other.getGoal().getGoal()
						.getTargetPose().getPose().getPosition().getY()
				&& this.getGoal().getGoal().getTargetPose().getPose()
						.getPosition().getZ() == other.getGoal().getGoal()
						.getTargetPose().getPose().getPosition().getZ()
				&& this.getGoal().getGoal().getTargetPose().getPose()
						.getOrientation().getW() == other.getGoal().getGoal()
						.getTargetPose().getPose().getOrientation().getW();
	}
}
