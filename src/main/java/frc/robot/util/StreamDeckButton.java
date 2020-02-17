package frc.robot.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class StreamDeckButton extends Trigger {
  private final StreamDeck streamdeck;
  private final int button;
  private String icon;
  private Command currentCommand;

  public StreamDeckButton(StreamDeck streamdeck, int button) {
    this.streamdeck = streamdeck;
    this.button = button;
    CommandScheduler.getInstance().onCommandFinish(command -> handleFinished(command));
  }

  public StreamDeckButton(StreamDeck streamdeck, int button, String icon) {
    this.streamdeck = streamdeck;
    this.button = button;
    setIcon(icon);
    CommandScheduler.getInstance().onCommandFinish(command -> handleFinished(command));
  }

  public String getIcon() {
    return this.icon;
  }

  public void setIcon(String icon) {
    this.icon = icon;
    this.streamdeck.setIcon(button, icon);
  }

  public void setAction(boolean action) {
    this.streamdeck.setAction(button, action);
  }

  @Override
  public boolean get() {
    return this.streamdeck.getAction(this.button);
  }

  public void setStatus(boolean status) {
    this.streamdeck.setStatus(button, status);
  }

  public boolean getStatus() {
    return this.streamdeck.getStatus(button);
  }

  private void handleFinished(Command command) {
    if (command == currentCommand) {
      setAction(false);
    }
  }

  /**
   * Starts the given command whenever the button is newly pressed.
   *
   * @param command       the command to start
   * @param interruptible whether the command is interruptible
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenPressed(final Command command, boolean interruptible) {
    whenActive(command, interruptible);
    currentCommand = command;
    return this;
  }

  /**
   * Starts the given command whenever the button is newly pressed. The command is set to be
   * interruptible.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenPressed(final Command command) {
    whenActive(command);
    currentCommand = command;
    return this;
  }

  /**
   * Runs the given runnable whenever the button is newly pressed.
   *
   * @param toRun        the runnable to run
   * @param requirements the required subsystems
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenPressed(final Runnable toRun, Subsystem... requirements) {
    whenActive(toRun, requirements);
    return this;
  }

  /**
   * Constantly starts the given command while the button is held.
   *
   * {@link Command#schedule(boolean)} will be called repeatedly while the button is held, and will
   * be canceled when the button is released.
   *
   * @param command       the command to start
   * @param interruptible whether the command is interruptible
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whileHeld(final Command command, boolean interruptible) {
    whileActiveContinuous(command, interruptible);
    currentCommand = command;
    return this;
  }

  /**
   * Constantly starts the given command while the button is held.
   *
   * {@link Command#schedule(boolean)} will be called repeatedly while the button is held, and will
   * be canceled when the button is released.  The command is set to be interruptible.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whileHeld(final Command command) {
    whileActiveContinuous(command);
    currentCommand = command;
    return this;
  }

  /**
   * Constantly runs the given runnable while the button is held.
   *
   * @param toRun        the runnable to run
   * @param requirements the required subsystems
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whileHeld(final Runnable toRun, Subsystem... requirements) {
    whileActiveContinuous(toRun, requirements);
    return this;
  }

  /**
   * Starts the given command when the button is first pressed, and cancels it when it is released,
   * but does not start it again if it ends or is otherwise interrupted.
   *
   * @param command       the command to start
   * @param interruptible whether the command is interruptible
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenHeld(final Command command, boolean interruptible) {
    whileActiveOnce(command, interruptible);
    currentCommand = command;
    return this;
  }

  /**
   * Starts the given command when the button is first pressed, and cancels it when it is released,
   * but does not start it again if it ends or is otherwise interrupted.  The command is set to be
   * interruptible.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenHeld(final Command command) {
    whileActiveOnce(command, true);
    currentCommand = command;
    return this;
  }

  /**
   * Starts the command when the button is released.
   *
   * @param command       the command to start
   * @param interruptible whether the command is interruptible
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenReleased(final Command command, boolean interruptible) {
    whenInactive(command, interruptible);
    currentCommand = command;
    return this;
  }

  /**
   * Starts the command when the button is released.  The command is set to be interruptible.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenReleased(final Command command) {
    whenInactive(command);
    currentCommand = command;
    return this;
  }

  /**
   * Runs the given runnable when the button is released.
   *
   * @param toRun        the runnable to run
   * @param requirements the required subsystems
   * @return this button, so calls can be chained
   */
  public StreamDeckButton whenReleased(final Runnable toRun, Subsystem... requirements) {
    whenInactive(toRun, requirements);
    return this;
  }

  /**
   * Toggles the command whenever the button is pressed (on then off then on).
   *
   * @param command       the command to start
   * @param interruptible whether the command is interruptible
   */
  public StreamDeckButton toggleWhenPressed(final Command command, boolean interruptible) {
    toggleWhenActive(command, interruptible);
    currentCommand = command;
    return this;
  }

  /**
   * Toggles the command whenever the button is pressed (on then off then on).  The command is set
   * to be interruptible.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton toggleWhenPressed(final Command command) {
    toggleWhenActive(command);
    currentCommand = command;
    return this;
  }

  /**
   * Cancels the command when the button is pressed.
   *
   * @param command the command to start
   * @return this button, so calls can be chained
   */
  public StreamDeckButton cancelWhenPressed(final Command command) {
    cancelWhenActive(command);
    currentCommand = command;
    return this;
  }

}