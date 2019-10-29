package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math; // headers MUST be above the first class
import java.util.concurrent.Callable;
import java.util.*;


/*
// one class needs to have a main() method
public class Main
{
	// arguments are passed using the text field below this editor
	public static void main(String[] args)
	{
		BreakableLoop loop = new BreakableLoop(new Callable<Void>() {
			@Override
			public Void call() {
				System.out.println("Hello");
				return null;
			}
		});
		loop.addBreak(new CountToFive());

		loop.run();
	}
}
*/

/**
 * Debouncing is an important for when you want to "throttle" how often a given block of code can be called.
 * In many instances there may be bad consequences from running a block of code too quickly after it has
 * already been run. Debouncing "wraps" a block of code so that instead of calling the code, you call the
 * debouncer. The debouncer keeps a running timer of when it was last called--meaning that it should or should
 * not be called again.
 *
 * Java doesn't have great syntax for how to do this kind of thing. The Callable interface works but is ugly.
 * Ideally, I'd like to get us to the point of being able to use Java 9, which has anonymous lambda functions.
 * These look a lot cleaner.
 */
class Debouncer {
	public long timeout;
	private ElapsedTime timer;
	private Callable codeBlock;
	private boolean isFirstRun = true;

	public Debouncer(long t, Callable<Void> block) {
		timeout = t;
		timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
		timer.reset();
		codeBlock = block;
	}

	public void run() throws Exception {
		if (isFirstRun || (timer.time() > timeout)) {
			isFirstRun = false;
			codeBlock.call();
			timer.reset();
		}
	}
}

class BreakableLoop {
	private ArrayList<BooleanTripwire> breaks = null;
	private Callable codeBlock = null;

	public BreakableLoop() {
		breaks = new ArrayList<BooleanTripwire>();
	}
	public BreakableLoop(Callable<Void> myBlock) {
		this();
		codeBlock = myBlock;
	}

	public void addBreak(BooleanTripwire b) {
		breaks.add(b);
	}
	public boolean anyTrue() {
		boolean result = false;
		Iterator<BooleanTripwire> it = breaks.iterator();
		while (it.hasNext()) {
			try {
				result = result || it.next().call();
			} catch (Exception e) {
				result = true;
				return result;
			}
		}
		return result;
	}

	public void run() {
		while (!anyTrue()) {
			if (codeBlock != null) {
				try {
					codeBlock.call();
				} catch (Exception e) {

				}
			} else {
				System.out.println("Loop");
			}
		}

	}
}

class BooleanTripwire implements Callable<Boolean> {

	@Override
	public Boolean call() throws Exception {
	 	return true;
	}
}

class CountToFive extends BooleanTripwire {
	private int count;

	CountToFive() {
		count = 0;
	}

	@Override
	public Boolean call() throws Exception {
		count += 1;
		return count > 5;
	}
}

class TimerTripwire extends BooleanTripwire {
	private ElapsedTime timer = null;
	private long duration;

	TimerTripwire(long d) {
		duration = d;
	}

	@Override
	public Boolean call() throws Exception {
		if (timer == null) {
			timer = new ElapsedTime();
			timer.reset();
		}
		return timer.milliseconds() > duration;
	}
}