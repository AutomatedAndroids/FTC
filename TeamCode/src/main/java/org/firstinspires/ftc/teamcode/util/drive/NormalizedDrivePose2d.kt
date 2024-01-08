package org.firstinspires.ftc.teamcode.util.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

data class NormalizedDrivePose2d(val x: Double, val y: Double, val heading: Double) {
    fun Pose2d.drivePowerNormalize(): NormalizedDrivePose2d {
        val root = Math.sqrt(Math.pow(this.x,2.0)+Math.pow(this.y,2.0)+Math.pow(this.heading,2.0))
        return NormalizedDrivePose2d(this.x/root, this.y/root, this.heading/root)
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