package org.firstinspires.ftc.teamcode.util.drive

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d

/** A vector system which has values less than one and greater than -1.*/
data class NormalizedDriveVector( val x: Double, val y: Double) {
    fun Vector2d.drivePowerNormalize(): NormalizedDriveVector {
        val root = Math.sqrt(Math.pow(this.x,2.0)+Math.pow(this.y,2.0))
        return NormalizedDriveVector(this.x/root, this.y/root)
    }
    operator fun NormalizedDriveVector.div(b: Double): NormalizedDriveVector {
        return NormalizedDriveVector(x/b,y/b)
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