package org.usfirst.frc.team5811.interfaces;

public interface BiDirectionalElement extends StoppableElement {
	public void set(double rate);
	public void inward();
	public void outward();
}
