//-----------------------------------------------------------------------------
// Torque
// Copyright GarageGames, LLC 2011
//-----------------------------------------------------------------------------

new Material(CommonPlayerFootprint)
{
   diffuseMap[0] = "footprint";
   vertColor[0] = true;
   translucent = true;
   castShadows = "0";
   translucentZWrite = "1";
   materialTag0 = "Player";
};

singleton Material(PlayerGibletBase)
{
   mapTo = "base.giblt";
   diffuseMap[0] = "art/shapes/actors/common/base.giblt";
   normalMap[0] = "art/shapes/actors/common/basegiblt_n";
   specularMap[0] = "art/shapes/actors/common/basegiblt_s.png";
   specular[0] = "1 0.8 0.9 1";
   specularPower[0] = 16;
   materialTag0 = "Player";
};
