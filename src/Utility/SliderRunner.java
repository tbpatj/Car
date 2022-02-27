package Utility;

import java.util.ArrayList;
import java.util.List;

import org.lwjgl.input.Mouse;
import org.lwjgl.opengl.Display;
import org.lwjgl.opengl.GL11;

public class SliderRunner {
	List<Slider> sliders = new ArrayList<Slider>();
	public void Run()
	{
		Mouse.setClipMouseCoordinatesToWindow(true);
		Mouse.updateCursor();
		float mouseX = Mouse.getX();
		float mouseY = Mouse.getY();
		mouseX = mouseX / Display.getWidth() * 1000;
		mouseY = mouseY / Display.getHeight() * 900;
		//System.out.println(mouseY);
		
		for(int i = 0; i < sliders.size(); i ++)
		{
			Slider slider = sliders.get(i);
			GL11.glBegin(GL11.GL_QUADS);
			GL11.glColor3f(0.9f, 0.9f, 0.9f);
			GL11.glVertex2f((slider.MaxRenderX) / 1000 * Display.getWidth(), (slider.MaxRenderY) / 900 * Display.getHeight());
			GL11.glColor3f(0.4f, 0.4f, 0.4f);
			GL11.glVertex2f((slider.MaxRenderX) / 1000 * Display.getWidth(), (slider.MinRenderY) / 900 * Display.getHeight());
			GL11.glColor3f(0.4f, 0.4f, 0.4f);
			GL11.glVertex2f((slider.MinRenderX) / 1000 * Display.getWidth(), (slider.MinRenderY) / 900 * Display.getHeight());
			GL11.glColor3f(0.9f, 0.9f, 0.9f);
			GL11.glVertex2f((slider.MinRenderX) / 1000 * Display.getWidth(), (slider.MaxRenderY) / 900 * Display.getHeight());
			
			GL11.glColor3f(0.4f, 0.4f, 0.4f);
			GL11.glVertex2f((slider.MaxRenderX - 5) / 1000 * Display.getWidth(), (slider.slideY + 2) / 900 * Display.getHeight());
			GL11.glColor3f(0.9f, 0.9f, 0.9f);
			GL11.glVertex2f((slider.MaxRenderX - 5) / 1000 * Display.getWidth(), (slider.slideY - 2) / 900 * Display.getHeight());
			GL11.glColor3f(0.9f, 0.9f, 0.9f);
			GL11.glVertex2f((slider.MinRenderX + 5) / 1000 * Display.getWidth(), (slider.slideY - 2) / 900 * Display.getHeight());
			GL11.glColor3f(0.4f, 0.4f, 0.4f);
			GL11.glVertex2f((slider.MinRenderX + 5) / 1000 * Display.getWidth(), (slider.slideY + 2) / 900 * Display.getHeight());
			
			
			GL11.glColor3f(1, 0, 0);
			GL11.glVertex2f((slider.slideX + 5) / 1000 * Display.getWidth(), (slider.slideY + 5) / 900 * Display.getHeight());
			GL11.glColor3f(0.4f, 0, 0);
			GL11.glVertex2f((slider.slideX + 5) / 1000 * Display.getWidth(), (slider.slideY - 5) / 900 * Display.getHeight());
			GL11.glColor3f(0.4f, 0, 0);
			GL11.glVertex2f((slider.slideX - 5) / 1000 * Display.getWidth(), (slider.slideY - 5) / 900 * Display.getHeight());
			GL11.glColor3f(1, 0, 0);
			GL11.glVertex2f((slider.slideX - 5) / 1000 * Display.getWidth(), (slider.slideY + 5) / 900 * Display.getHeight());
			GL11.glEnd();
			if(Mouse.isButtonDown(0))
			{
				if(mouseX < slider.slideX + 5 && mouseX > slider.slideX - 5 && mouseY < slider.slideY + 5 && mouseY > slider.slideY - 5)
				{
					slider.pickedUp = true;
				}
				
				if(slider.pickedUp)
				{
					if(slider.horizontal)
					{
						slider.slideX = mouseX;
						if(slider.slideX > slider.MaxRenderX - 5)
						{
							slider.slideX = slider.MaxRenderX - 5;
						}
						if(slider.slideX < slider.MinRenderX + 5)
						{
							slider.slideX = slider.MinRenderX + 5;
						}
						slider.value = (slider.slideX - slider.posX);
						if(slider.dividedBy)
						{
							slider.value = slider.value / slider.divide;
						}
						if(slider.mulitiplied)
						{
							slider.value = slider.value * slider.multiply;
						}
					}
					else
					{
						slider.slideY = mouseY;
						slider.value = (slider.slideY - slider.posY);
					}
					
				}
			}
			else
			{
				slider.pickedUp = false;
			}
			
			
			
		}
	}
	public void sliderSetValue(int index)
	{
		Slider slide = sliders.get(index);
		slide.slideX = slide.posX;
		slide.slideY = slide.posY;
		slide.value = 0;
	}
	public void createSlider(float x, float y, float maxRenX, float maxRenY, float minRenX, float minRenY, boolean horizontal, float divide, float multiply)
	{
		Slider slide = new Slider();
		slide.posX = x;
		slide.posY = y;
		slide.MaxRenderX = maxRenX;
		slide.MaxRenderY = maxRenY;
		slide.MinRenderX = minRenX;
		slide.MinRenderY = minRenY;
		slide.horizontal = horizontal;
		slide.maximum = 100;
		slide.minimum = 100;
		slide.slideX = x;
		slide.slideY = y;
		slide.value = 0;
		if(divide != 0)
		{
			slide.dividedBy = true;
			slide.divide = divide;
		}
		if(multiply != 0)
		{
			slide.mulitiplied = true;
			slide.multiply = multiply;
		}
		sliders.add(slide);
	}
	public float returnSliderValue(int i)
	{
		Slider slide = sliders.get(i);
		return slide.value;
	}
}
