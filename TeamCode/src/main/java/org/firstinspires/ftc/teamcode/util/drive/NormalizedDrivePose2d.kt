package org.firstinspires.ftc.teamcode.util.drive

import com.acmerobotics.roadrunner.geometry.Pose2d

data class NormalizedDrivePose2d(var x: Double, var y: Double, var heading: Double) {
    var root: Double = 1.0
//

    companion object {
        fun drivePowerNormalize(pose: Pose2d): NormalizedDrivePose2d {
            val root = Math.sqrt(Math.pow(pose.x,2.0)+Math.pow(pose.y,2.0)+Math.pow(pose.heading,2.0))
            return NormalizedDrivePose2d(pose.x/root, pose.y/root, pose.heading/root)
        }
        fun deNormalizePose(normalizedPose: NormalizedDrivePose2d): Pose2d {
            return Pose2d(normalizedPose.x*normalizedPose.root,normalizedPose.y*normalizedPose.root,normalizedPose.heading*normalizedPose.root)
        }
    }

    fun drivePowerNormalize(pose: Pose2d): NormalizedDrivePose2d {
        this.root = Math.sqrt(Math.pow(pose.x,2.0)+Math.pow(pose.y,2.0)+Math.pow(pose.heading,2.0))
        this.x = pose.x/root
        this.y = pose.y/root
        this.heading = pose.heading/root
        return this
    }
    fun deDriveNormalize(): Pose2d {
        return Pose2d(x*root,y*root,heading*root)
    }

    operator fun NormalizedDrivePose2d.div(b: Double): NormalizedDrivePose2d {
        return NormalizedDrivePose2d(x/b,y/b,heading/b)
    }
    operator fun NormalizedDrivePose2d.times(b: Double): NormalizedDrivePose2d {
        return NormalizedDrivePose2d(x*b,y*b,heading*b)
    }
    operator fun Pose2d.div(b: Double): Pose2d {
        return Pose2d(this.x/b,this.y/b,this.heading/b)
    }
    operator fun Pose2d.times(b: Double): Pose2d {
        return Pose2d(this.x*b,this.y*b,this.heading*b)
    }
}