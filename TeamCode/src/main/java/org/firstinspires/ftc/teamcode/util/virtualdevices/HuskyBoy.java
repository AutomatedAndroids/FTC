package org.firstinspires.ftc.teamcode.util.virtualdevices;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.teamcode.Hardware;

import java.util.ArrayList;

import kotlin.text.CharDirectionality;

public class HuskyBoy {
    HuskyLens husky;
    public HuskyBoy(Hardware hardware) {
        this.husky = hardware.huskyLens;
    }
    HuskyBoy(HuskyLens huskyLens) {
        this.husky = huskyLens;
    }
    // Constants
    public static int leftPixelBound;
    public static int rightPixelBound;

    enum Tag{
        BlueAlliLeft,
        BlueAlliCenter,
        BlueAlliRight,
        RedAlliLeft,
        RedAlliCenter,
        RedAlliRight,
        AudienceLeft,
        AudienceRight,
        AprilTagGeneric
    }

    enum Prop {
        BlueTeamOur,
        RedTeamOur,
        BlueTeamTheirs,
        RedTeamTheirs,
        PropGeneric
    }
    public enum PropLocation {
        LEFT, CENTER, RIGHT
    }

    /** This function will return an arraylist of all tags seen by the robot.
     * It uses the Tag enumerator. */
    public ArrayList<Tag> scanTag() {

       husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        ArrayList<Tag> tags = new ArrayList<Tag>();
        if (husky.blocks() == null) {
            return new ArrayList<Tag>();
        } else {
            for (HuskyLens.Block block : husky.blocks()) {
                if (block.id < 0) {
                    tags.add(Tag.AprilTagGeneric);
                } else {
                    tags.add(Tag.values()[block.id - 1]);
                }
            }
        }
        return tags;
    }
    /** This function will return an arraylist of all props seen by the robot.
     * Preferably, the list will only be a length of 1, however to prevent the
     * wrong prop from being recognized, it returns all scene by the huskylens
     * assuming see multiple is enabled.*/
    public ArrayList<Prop> scanProp() {
        husky.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION);
        ArrayList<Prop> props = new ArrayList<Prop>();
        if (husky.blocks() == null) {
            return new ArrayList<Prop>();
        } else {
            for (HuskyLens.Block block : husky.blocks()) {
                props.add(Prop.values()[block.id-1]);
            }
        }
        return props;
    }
    public PropLocation findPropLocation(HuskyLens.Block block) {
        if (block.x < leftPixelBound) {
            return PropLocation.LEFT;
        } else if (block.x < rightPixelBound && block.x >= leftPixelBound) {
            return PropLocation.CENTER;
        } else {
            return PropLocation.RIGHT;
        }
    }

    public HuskyLens getHusky() {
        return husky;
    }
}
