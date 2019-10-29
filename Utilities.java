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