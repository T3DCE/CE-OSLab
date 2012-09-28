//-----------------------------------------------------------------------------
// Torque
// Copyright GarageGames, LLC 2011
//-----------------------------------------------------------------------------

function BulletSettings::onWake()
{
   if(!$Pref::Bullet::maxSubSteps)
      $Pref::Bullet::maxSubSteps = 2;

   sldmaxSubSteps.value = $Pref::Bullet::maxSubSteps;
   
   switch($Pref::Bullet::Method) 
   {
      case 0:
         radio60Hz.setStateOn(true);
         sldfixedTimeStep.active = false;
         $Pref::Bullet::fixedTimeStep = 0.017;
         sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
         sldmaxSubSteps.value = $Pref::Bullet::maxSubSteps;
      case 1:
         radioAutomatic.setStateOn(true);
         sldfixedTimeStep.active = false;
         sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
         sldmaxSubSteps.value = $Pref::Bullet::maxSubSteps;
      case 2:
         radioManual.setStateOn(true);
          sldfixedTimeStep.active = true;
          sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
          sldmaxSubSteps.value = $Pref::Bullet::maxSubSteps;
      default:
         radio60Hz.setStateOn(true);
         $Pref::Bullet::Method = 0;
         $Pref::Bullet::fixedTimeStep = 0.017;
         sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
         sldmaxSubSteps.value = 2;
         sldfixedTimeStep.active = false;
   }

}

function setFixedTimeStep(%value)
{
   if ($Pref::Bullet::Method > 1)
   {  
      $Pref::Bullet::fixedTimeStep = %value;
      sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
   }
}

function setMaxSubSteps(%value)
{
   $Pref::Bullet::maxSubSteps = %value;
   sldmaxSubSteps.value = $Pref::Bullet::maxSubSteps;
}

function fixedTimeStep_selector()  
{  
    %m60Hz = radio60Hz.getValue();  
    %mAutomatic = radioAutomatic.getValue();  
    %mManual = radioManual.getValue();  
     
   if(%m60Hz)  
   {  
      sldfixedTimeStep.active = false; 
      $Pref::Bullet::fixedTimeStep = 0.017;
      sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
      $Pref::Bullet::Method = 0;
   }  
   else if(%mAutomatic)  
   {  
      sldfixedTimeStep.active = false;
      $Pref::Bullet::fixedTimeStep = ($fps::real / 1000);
      sldfixedTimeStep.value = $Pref::Bullet::fixedTimeStep;
      $Pref::Bullet::Method = 1;  
   }  
   else if(%mManual)  
   {  
      sldfixedTimeStep.active = true;
      $Pref::Bullet::Method = 2;    
   }  
}  