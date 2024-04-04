// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lights extends SubsystemBase {

  private CANdle candle = new CANdle(Constants.candleID);

  
  private final int underGlowLedCount = 91;
  private final int rightClimberLEDCount = 58;
  private final int leftClimberLEDCount = 59;
  private final int toldLEDCount = underGlowLedCount + rightClimberLEDCount + leftClimberLEDCount;

  private CANdleConfiguration cfg = new CANdleConfiguration();

  private Animation m_toAnimate = null;

  

    public enum AnimationTypes {
        ColorFlow,
        Larson,
        Rainbow,
        RgbFade,
        SingleFade,
        Twinkle,
        TwinkleOff,
        SetAll
    }

    private AnimationTypes m_currentAnimation;

    private Feeder feeder;

    private Shooter shooter;

  /** Creates a new Lights. */
  public Lights(Shooter shooter, Feeder feeder) {
    this.feeder = feeder;
    this.shooter = shooter;
    cfg.statusLedOffWhenActive = true;
    cfg.stripType = LEDStripType.GRB;
    //cfg.brightnessScalar = .1;
    cfg.disableWhenLOS = false;
    
    candle.configFactoryDefault();
    candle.clearStickyFaults();
    candle.configAllSettings(cfg);
    setColors();
    changeAnimation(AnimationTypes.SetAll);

    
  }

  public void incrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.Larson); break;
            case Larson: changeAnimation(AnimationTypes.Rainbow); break;
            case Rainbow: changeAnimation(AnimationTypes.RgbFade); break;
            case RgbFade: changeAnimation(AnimationTypes.SingleFade); break;
            case SingleFade: changeAnimation(AnimationTypes.Twinkle); break;
            case Twinkle: changeAnimation(AnimationTypes.TwinkleOff); break;
            case TwinkleOff: changeAnimation(AnimationTypes.ColorFlow); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }
    public void decrementAnimation() {
        switch(m_currentAnimation) {
            case ColorFlow: changeAnimation(AnimationTypes.TwinkleOff); break;
            case Larson: changeAnimation(AnimationTypes.ColorFlow); break;
            case Rainbow: changeAnimation(AnimationTypes.Larson); break;
            case RgbFade: changeAnimation(AnimationTypes.Rainbow); break;
            case SingleFade: changeAnimation(AnimationTypes.RgbFade); break;
            case Twinkle: changeAnimation(AnimationTypes.SingleFade); break;
            case TwinkleOff: changeAnimation(AnimationTypes.Twinkle); break;
            case SetAll: changeAnimation(AnimationTypes.ColorFlow); break;
        }
    }

  public void setColors() {
        changeAnimation(AnimationTypes.SetAll);
    }

  public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case ColorFlow:
                if (isBlue()) {
                    m_toAnimate = new ColorFlowAnimation(0, 0, 255, 0, 0.7, underGlowLedCount, Direction.Forward);
                } else {
                    m_toAnimate = new ColorFlowAnimation(255, 0, 0, 0, 0.7, underGlowLedCount, Direction.Forward);
                }
                break;
            
            case Larson:
                if (isBlue()) {
                    m_toAnimate = new LarsonAnimation(0, 0, 255, 0, .4, underGlowLedCount, BounceMode.Front, 5);
                } else {
                    m_toAnimate = new LarsonAnimation(255, 0, 0, 0, .4, underGlowLedCount, BounceMode.Front, 5);
                }
                break;
            case Rainbow:
                m_toAnimate = new RainbowAnimation(0.7, 0.6, rightClimberLEDCount, false, underGlowLedCount+1);
                break;
            case RgbFade:
                m_toAnimate = new RgbFadeAnimation(0.7, 0.6, underGlowLedCount);
                break;
            case SingleFade:
                if (isBlue()) {
                    m_toAnimate = new SingleFadeAnimation(0, 0, 200, 0, 0.5, underGlowLedCount);
                } else {
                    m_toAnimate = new SingleFadeAnimation(200, 0, 0, 0, 0.5, underGlowLedCount);
                }
                break;
            case Twinkle:
                m_toAnimate = new TwinkleAnimation(30, 70, 60, 0, 0.4, underGlowLedCount, TwinklePercent.Percent30);
                break;
            case TwinkleOff:
                m_toAnimate = new TwinkleOffAnimation(70, 90, 175, 0, 0.6, underGlowLedCount, TwinkleOffPercent.Percent100);
                break;
            case SetAll:
                m_toAnimate = null;
                break;
        }
        //System.out.println("Changed to " + m_currentAnimation.toString());
    }

  public boolean isBlue() {
    try {
        if (DriverStation.getAlliance().get().equals(Alliance.Blue)) {
            return true;
        } else {
            return false;
        }
    } catch (Exception e) {
            //e.printStackTrace();     
            return false;      
    }
  }

  public void setEveryOtherLED(int r, int g, int b, double ledCount) {
    for(int i = 0; i < ledCount; i+=2) {
        
        candle.setLEDs(r, g, b, 0, i, 1);
    }
  }

    

  @Override
  public void periodic() {
    candle.animate(null);

    if (!DriverStation.isDSAttached()) {
        //changeAnimation(AnimationTypes.SingleFade);
    //} else if (DriverStation.isDSAttached() && DriverStation.isDisabled()) {
        //changeAnimation(AnimationTypes.Rainbow);
    } else if (feeder.isfeedStopped() || shooter.isShooterUpToSpeed()) {
        setColors();
        candle.setLEDs(0, 255, 0, 0, 0, rightClimberLEDCount+leftClimberLEDCount);
    } else {
        setColors();
        if (isBlue()) { 
            candle.setLEDs(0, 0, 255, 0, 0, rightClimberLEDCount+leftClimberLEDCount);
            //setEveryOtherLED(255, 0, 0, underGlowLedCount);
        } else {
            candle.setLEDs(255, 0, 0, 0, 0, rightClimberLEDCount+leftClimberLEDCount);
            //setEveryOtherLED(0, 0, 255, underGlowLedCount);
        }
    }
    
    
    SmartDashboard.putString("animation", m_currentAnimation.toString());
    // This method will be called once per scheduler run
    if(m_toAnimate == null) {
        candle.animate(null);
            
        } else {
            candle.animate(m_toAnimate);
        }
  }
}
