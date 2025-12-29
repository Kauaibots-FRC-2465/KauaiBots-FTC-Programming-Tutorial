package org.firstinspires.ftc.teamcode;

import com.seattlesolvers.solverslib.command.FunctionalCommand;
import com.seattlesolvers.solverslib.command.Subsystem;

public class OverrideCommand extends FunctionalCommand {
    public OverrideCommand(Subsystem... requirements) {
        super (()->{},
                ()->{},
                ( interrupted)->{},
                ()->false, requirements);
    }
}