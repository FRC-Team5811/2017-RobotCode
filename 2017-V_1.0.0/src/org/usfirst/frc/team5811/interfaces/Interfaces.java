package org.usfirst.frc.team5811.interfaces;

interface StoppableElement {
	public void stop();
}

interface BiDirectionalElement extends StoppableElement {
	public void inward();
	public void outward();
}

