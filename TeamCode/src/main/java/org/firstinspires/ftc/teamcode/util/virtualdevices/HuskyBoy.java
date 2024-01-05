package org.firstinspires.ftc.teamcode.util.virtualdevices;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import java.util.ArrayList;

public class HuskyBoy {
    HuskyLens husky;
    HuskyBoy(HuskyLens huskyLens) {
        this.husky = huskyLens;
    }

    enum Tag{
        BlueAlliLeft,
        BlueAlliCenter,
        BlueAlliRight,
        RedAlliLeft,
        RedAlliCenter,
        RedAlliRight,
        AudienceLeft,
        AudienceRight
    }

    enum Prop {
        BlueTeamOur,
        RedteamOur,
        BlueTeamTheirs,
        RedTeamTheirs
    }

    /** This function will return an arraylist of all tags seen by the robot.
     * It uses the Tag enumerator. */
    public ArrayList<Tag> scanTag() {
        husky.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
        ArrayList<Tag> tags = new ArrayList<Tag>();
        if (husky.blocks()[0] == null) {
            return new ArrayList<Tag>();
        }
        for (HuskyLens.Block block : husky.blocks()) {
            tags.add(Tag.values()[block.id]);
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
        if (husky.blocks()[0] == null) {
            return new ArrayList<Prop>();
        }
        for (HuskyLens.Block block : husky.blocks()) {
            props.add(Prop.values()[block.id]);
        }
        return props;
    }


}
