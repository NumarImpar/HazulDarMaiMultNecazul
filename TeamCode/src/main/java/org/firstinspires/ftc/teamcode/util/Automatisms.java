package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Lifter;

import org.firstinspires.ftc.teamcode.mechanisms.Lifter.LIFTER_LEVEL;
import org.firstinspires.ftc.teamcode.mechanisms.Intake.INTAKE_STATE;
import org.firstinspires.ftc.teamcode.teleop.Drive;

public class Automatisms {
    public Intake intake;
    public Lifter lifter;

    public Automatisms(Lifter _lifter, Intake _intake) {
        this.lifter = _lifter;
        this.intake = _intake;
    }

    public void goToState(LIFTER_LEVEL currentLifterState, INTAKE_STATE currentIntakeState, LIFTER_LEVEL newLifterState,
                          INTAKE_STATE newIntakeState) {
        if (newLifterState == Lifter.LIFTER_LEVEL.DOWN && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: DOWN IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                lifter.setTargetTicks(0, LIFTER_LEVEL.DOWN.ticks);
                intake.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                return;
            }

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.INSIDE) {
                // FROM: HIGH IN
                lifter.setTargetTicks(0, LIFTER_LEVEL.DOWN.ticks);
                return;
            }

            if (currentLifterState == LIFTER_LEVEL.MID && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: MID OUT
                lifter.setTargetTicks(0, newLifterState.ticks);
                intake.swingSetTargetTicks(200, newIntakeState.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.HIGH && newIntakeState == INTAKE_STATE.OUTSIDE) {
            // TO: HIGH OUT

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intake.swingSetTargetTicks(500, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.HIGH.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.HIGH && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: HIGH IN

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intake.swingSetTargetTicks(300, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(50, LIFTER_LEVEL.HIGH.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.MID && newIntakeState == INTAKE_STATE.OUTSIDE) {
            // TO: MID OUT

            if (currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (or INIT)
                intake.swingSetTargetTicks(300, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.MID && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: MID IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                intake.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.LOW && newIntakeState == INTAKE_STATE.INSIDE) {
            // TO: LOW IN

            if (currentLifterState == LIFTER_LEVEL.HIGH && currentIntakeState == INTAKE_STATE.OUTSIDE) {
                // FROM: HIGH OUT
                intake.swingSetTargetTicks(0, INTAKE_STATE.INSIDE.ticks);
                lifter.setTargetTicks(0, LIFTER_LEVEL.LOW.ticks);
                return;
            }
        }

        if (newLifterState == LIFTER_LEVEL.DOWN && newIntakeState == INTAKE_STATE.OUTSIDE){
            // TO: DOWN OUT

            if(currentLifterState == LIFTER_LEVEL.DOWN && (currentIntakeState == INTAKE_STATE.INSIDE || currentIntakeState == INTAKE_STATE.INIT)) {
                // FROM: DOWN IN (OR INIT)
                lifter.setTargetTicks(0, LIFTER_LEVEL.MID.ticks);
                intake.swingSetTargetTicks(500, INTAKE_STATE.OUTSIDE.ticks);
                lifter.setTargetTicks(800, LIFTER_LEVEL.DOWN.ticks);
                return;
            }
        }

        // no specific automation, just go there (manual handling)
        intake.swingSetTargetTicks(0, newIntakeState.ticks);
        lifter.setTargetTicks(0, newLifterState.ticks);
    }
}

