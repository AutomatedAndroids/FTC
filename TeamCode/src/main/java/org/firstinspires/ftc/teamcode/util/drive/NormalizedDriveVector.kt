package org.firstinspires.ftc.teamcode.util.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

/** A vector system which has values less than one and greater than -1.*/
data class NormalizedDriveVector( var x: Double, var y: Double) {
    var root: Double = 1.0
    companion object {
        fun drivePowerNormalize(pose: Pose2d): NormalizedDriveVector {
            val root = Math.sqrt(Math.pow(pose.x,2.0)+Math.pow(pose.y,2.0))
            return NormalizedDriveVector(pose.x/root, pose.y/root)
        }
        fun deNormalizePose(normalizedPose: NormalizedDrivePose2d): Vector2d {
            return Vector2d(normalizedPose.x*normalizedPose.root,normalizedPose.y*normalizedPose.root)
        }
    }
//    fun Vector2d.drivePowerNormalize(): NormalizedDriveVector {
//        val root = Math.sqrt(Math.pow(this.x,2.0)+Math.pow(this.y,2.0))
//        return NormalizedDriveVector(this.x/root, this.y/root)
//    }
    fun drivePowerNormalize(vector: Vector2d): NormalizedDriveVector {
        this.root = Math.sqrt(Math.pow(vector.x,2.0)+Math.pow(vector.y,2.0))
        this.x = vector.x/root
        this.y = vector.y/root
        return this
    }
    operator fun NormalizedDriveVector.div(b: Double): NormalizedDriveVector {
        return NormalizedDriveVector(x/b,y/b)
    }
    fun deDriveNormalize(): Vector2d {
        return Vector2d(x*root,y*root)
    }
    operator fun NormalizedDrivePose2d.times(b: Double): NormalizedDriveVector {
        return NormalizedDriveVector(x*b,y*b)
    }
    operator fun Vector2d.div(b: Double): Vector2d {
        return Vector2d(this.x/b,this.y/b)
    }
    operator fun Vector2d.times(b: Double): Vector2d {
        return Vector2d(this.x*b,this.y*b)
    }
}