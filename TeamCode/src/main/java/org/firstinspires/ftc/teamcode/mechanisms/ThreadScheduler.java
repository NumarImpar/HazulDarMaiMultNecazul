package org.firstinspires.ftc.teamcode.mechanisms;

import java.util.ArrayList;
import java.util.ListIterator;

public class ThreadScheduler extends ArrayList<Thread>{
	public ThreadScheduler(){
		super();
	}

	private void freeMemory(){
		ListIterator<Thread> iter = this.listIterator();
		int index = 0;

		while(iter.hasNext()){
			Thread t = iter.next();
			if(! t.isAlive()){
				this.remove(index);
			}

			index += 1;
		}
	}

	public void newThread(Thread t){
		freeMemory();
		this.add(t);
		this.get(this.size() - 1).start();
	}

	public void killAll(){
		ListIterator<Thread> iter = this.listIterator();
		while(iter.hasNext()){
			iter.next().interrupt();
		}
		this.clear();
	}
}
