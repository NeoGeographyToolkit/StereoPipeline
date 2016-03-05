<?xml version="1.0"?>

<!-- 
Id : SSD1
Description : Stylesheet for SPOTScene products
Version : 2
-->

<xsl:stylesheet version="1.0" xmlns:xsl="http://www.w3.org/1999/XSL/Transform">
	<xsl:template match="/">
		<html>
	
			<head>
				<meta http-equiv="content-type" content="text/html;charset=ISO-8859-1"/>
				<title>dimap</title>
				<style type="text/css">
					.entete { color: #fff; font-style: italic; font-weight: bolder; font-size: 14px; font-family: Georgia, "Times New Roman" }
					.titre  { font-weight: bolder; font-size: 20px; font-family: Georgia, "Times New Roman"; text-align: center }
					.spotview { color: white; font-weight: bolder; font-size: 32px; font-family: Verdana, "Times New Roman"; text-align: center }
					.intitule { font-weight: bolder; font-size: 11px; font-family: Verdana, Arial }
					.intitule2 { color: silver; font-weight: bolder; font-size: 11px; font-family: Verdana, Arial }
					.description { font-size: 11px; font-family: Verdana, Arial }
					.affiche { display : show }
					.masque { display  : none }
			
					.onglets   {  font-weight: bolder; font-size: 11px; font-family: Verdana, Helvetica; text-decoration: none; text-align: center }
					.ongletsOn {  font-weight: bolder; font-size: 11px; font-family: Verdana, Helvetica; text-decoration: none; text-align: center }
			
					a.onglets    { color: #00607a; text-decoration: none; }
					a.onglets:active  { color: #00607a }
					a.onglets:hover  { color: white; text-decoration: none }
			
					a.ongletsOn    { color: #000000; text-decoration: none }
					a.ongletsOn:active  { color: #000000 }
					a.ongletsOn:hover  { color: #000000; text-decoration: none }
			
					div.onglets {background-color : #bbbbbb; }
					div.ongletsOn {background-color : #e4decb; }
			
					td.onglets {background-color : #bbbbbb; border-style : solid; border-width : 1pt; border-color : black;}
					td.ongletsOn {background-color : #e4decb;  border-style : solid; border-width : 1pt; border-color : black;}
			
					.ROI { border-style : thin-solid; border-color : white;}
					.ROIshow {  border-left : white 1px solid ; border-right : white 1px solid; border-bottom : white 1px solid ; border-top : white 1px solid ; display : show; visibility : visible }
				</style>
			
				<script language="javascript">
					var strPreviousOn = "description" ;
			
					function releaseFunc(strFunc) {
						var str = "divO_" + strFunc;
						var divelt = document.getElementById(str);
						divelt.className="onglets";
						
						str = "tdO_" + strFunc;
						divelt = document.getElementById(str);
						divelt.className="onglets";
						
						str = "aO_" + strFunc;
						divelt = document.getElementById(str);
						divelt.className="onglets";
					}
			
					function selectFunc(strFunc) {
						var str = "divO_" + strFunc;
						var divelt = document.getElementById(str);
						divelt.className="ongletsOn";
						
						str = "tdO_" + strFunc;
						divelt = document.getElementById(str);
						divelt.className="ongletsOn";
						
						str = "aO_" + strFunc;
						divelt = document.getElementById(str);
						divelt.className="ongletsOn";
			
						strPreviousOn = strFunc;
					}
				
					function details(strId) {
						var divelt = document.getElementById(strId);
						if (divelt.className=="masque") {
							divelt.className="affiche";
						}
						else {
							divelt.className="masque";
						}
					}
			
					function expandSources() {
						<xsl:for-each select="//Source_Information">
							eval('div<xsl:value-of select="position()"/>.className = \'affiche\'');
						</xsl:for-each>
					}
			
					function onWindowResize(e) {
			//			if (document.readyState == "loaded") {
							organize(strPreviousOn);
			//			}
					}
						
					// Taille de la scene
					function calcSceneSize() {
						frame_w = 0;
						frame_h = 0;
						<xsl:for-each select="//Dataset_Sources/Source_Frame/Vertex">
							frame_w = Math.max(frame_w, <xsl:value-of select="FRAME_COL"/>);
							frame_h = Math.max(frame_h, <xsl:value-of select="FRAME_ROW"/>);
						</xsl:for-each> 
					}
			
				
					function organize(strMode) {
					
						switch(strMode) {
							case "description":
								document.getElementById("divGenInfo").className  = "affiche";
								document.getElementById("divImgDim").className = "affiche";
								document.getElementById("divDtstFrm").className = "affiche";
								document.getElementById("divSources").className = "masque";
								document.getElementById("divCRS").className = "masque";
								document.getElementById("divGEO").className = "masque";
								document.getElementById("divProd").className = "affiche";
								document.getElementById("divQuicklook").className  = "masque";
								break;
					
							case "quicklook":
								document.getElementById("divGenInfo").className  = "masque";
								document.getElementById("divImgDim").className = "masque";
								document.getElementById("divDtstFrm").className = "masque";
								document.getElementById("divSources").className = "masque";
								document.getElementById("divCRS").className = "masque";
								document.getElementById("divGEO").className = "masque";
								document.getElementById("divProd").className = "masque";
								document.getElementById("divQuicklook").className  = "affiche";
								updateROIs();
								break;
			
							case "lineage":
								document.getElementById("divGenInfo").className  = "masque";
								document.getElementById("divImgDim").className = "masque";
								document.getElementById("divDtstFrm").className = "masque";
								document.getElementById("divCRS").className = "masque";
								document.getElementById("divGEO").className = "masque";
								document.getElementById("divQuicklook").className  = "masque";
								document.getElementById("divProd").className = "affiche";
								document.getElementById("divSources").className = "affiche";
								break;
			
							case "crs":
								document.getElementById("divGenInfo").className  = "masque";
								document.getElementById("divImgDim").className = "masque";
								document.getElementById("divDtstFrm").className = "masque";
								document.getElementById("divSources").className = "masque";
								document.getElementById("divCRS").className = "affiche";
								document.getElementById("divGEO").className = "affiche";
								document.getElementById("divProd").className = "masque";
								document.getElementById("divQuicklook").className  = "masque";
								break;
			
							case "globalinfo":
								document.getElementById("divGenInfo").className  = "affiche";
								document.getElementById("divImgDim").className = "affiche";
								document.getElementById("divDtstFrm").className = "affiche";
								document.getElementById("divSources").className = "affiche";
								document.getElementById("divCRS").className = "affiche";
								document.getElementById("divGEO").className = "affiche";
								document.getElementById("divProd").className = "affiche";
								document.getElementById("divQuicklook").className  = "affiche";
								updateROIs();	
								break;
					
							default:
								return false;
								break;
						}
					
						releaseFunc(strPreviousOn);
						selectFunc(strMode);
						
						updateROIQc();
					}				
				
					function updateROIs() {
						x = 0;
						y = 0;
						obj = document.getElementById("imgQl");
						while (obj.offsetParent != null) {
							x += obj.offsetLeft;
							y += obj.offsetTop;
							obj = obj.offsetParent;
						}
						x += obj.offsetLeft;
						y += obj.offsetTop;
						
						// Taille de la scene d'origine
						calcSceneSize();
				
						// Calcul de la position et de la taille de l'emprise sur le quicklook		
						<xsl:for-each select="//Regions_Of_Interest/Region_Of_Interest">
							roi = roi<xsl:value-of select="position()"/>;
							roi.style.position = "absolute";
							roi.className = "ROIshow";
							
							roi.style.left =  x + <xsl:value-of select="COL_MIN"/> * imgQl.width / frame_w + 1;
							roi.style.top =  y + <xsl:value-of select="ROW_MIN"/> * imgQl.height / frame_h + 1; 
							roi.style.width = (<xsl:value-of select="COL_MAX"/> - <xsl:value-of select="COL_MIN"/>) * imgQl.width / frame_w + 1;
							roi.style.height = (<xsl:value-of select="ROW_MAX"/> - <xsl:value-of select="ROW_MIN"/>) * imgQl.height / frame_h + 1;
							
						</xsl:for-each> 
					}
					
					function updateROIQc() {	
						x = 0;
						y = 0;
						obj = document.getElementById("imgQc");
						
						while (obj.offsetParent != null) {
							x += obj.offsetLeft;
							y += obj.offsetTop;
							obj = obj.offsetParent;
						}
						x += obj.offsetLeft;
						y += obj.offsetTop;
				
						// Taille de la scene d'origine
						calcSceneSize();
						
						// Gestion de la marge autour de l'icone
						var imaQl = new Image();
						imaQl.src =  "preview.jpg";
						hQC = 128 * imaQl.height / 1000;
						
						// Calcul de la position et de la taille de l'emprise sur l'icone		
						<xsl:for-each select="//Regions_Of_Interest/Region_Of_Interest">
							roi = qcroi<xsl:value-of select="position()"/>;
							roi.style.position = "absolute";
							roi.className = "ROIshow";
						
							roi.style.left =  x + <xsl:value-of select="COL_MIN"/> * imgQc.width / frame_w + 1;
							roi.style.top =  (y + <xsl:value-of select="ROW_MIN"/> * imgQc.height / frame_h) + ((imgQc.height - hQC) / 2) + 1; 
							roi.style.width = (<xsl:value-of select="COL_MAX"/> - <xsl:value-of select="COL_MIN"/>) * imgQc.width / frame_w + 1;
							roi.style.height = (<xsl:value-of select="ROW_MAX"/> - <xsl:value-of select="ROW_MIN"/>) * imgQc.height * imaQl.height / 1000 / frame_h + 1;
						</xsl:for-each> 
					}					
				</script>
			</head>
	
			<body bgcolor="#e4decb" onload="organize('description')" onresize="onWindowResize()">
				<div align="center">
					<table border="0" cellpadding="0" cellspacing="0" id="tblOnglets" width="80%">
						<tr height="20">
							<td width="100" border="1" height="20" id="tdO_description">
								<div id="divO_description" class="ongletsOn">
									<a id="aO_description" class="ongletsOn" href="javascript:organize('description')">Description</a>
								</div>
							</td>
							<td width="10" border="1" height="20"></td>
							<td width="100" height="20" id="tdO_lineage" class="onglets">
								<div id="divO_lineage" class="onglets">
									<a id="aO_lineage" class="onglets" href="javascript:organize('lineage')">Lineage</a>
								</div>
							</td>
							<td width="10" height="20"></td>
							<td width="140" height="20" id="tdO_crs" class="onglets">
								<div id="divO_crs" class="onglets">
									<a id="aO_crs" class="onglets" href="javascript:organize('crs')">Coordinate System</a>
								</div>
							</td>
							<td width="10" height="20"></td>
							<td width="100" height="20" id="tdO_quicklook" class="onglets">
								<div id="divO_quicklook" class="onglets">
									<a id="aO_quicklook" class="onglets" href="javascript:organize('quicklook')">Quicklook</a>
								</div>
							</td>
							<td width="40" height="20"></td>
							<td width="150" height="20" id="tdO_globalinfo" class="onglets">
								<div id="divO_globalinfo" class="onglets">
									<a id="aO_globalinfo" class="onglets" href="javascript:organize('globalinfo')">Printable information</a>
								</div>
							</td>
							<td width="10" height="20"></td>
						</tr>
						<tr height="5"></tr>
					</table>
				</div>
				<div align="center">
					<table border="3" bordercolor="black" cellpadding="0" cellspacing="0" width="80%">
						<tr>
							<td>
								<table border="0" cellpadding="0" cellspacing="0" width="100%">
									<tr>
										<td>
											<table border="0" cellpadding="0" cellspacing="0" width="100%">
												<tr>
													<td width="120">
														<div align="center">
															<a target="_blank" title="quicklook">
																<xsl:attribute name="href">
																	<xsl:value-of select="//DATASET_QL_PATH/@href"/>
																</xsl:attribute>
																<img border="0" id="imgQc">
																	<xsl:attribute name="src">
																		<xsl:value-of select="//DATASET_TN_PATH/@href"/>
																	</xsl:attribute>
																</img>
															</a>
															<xsl:for-each select="//Regions_Of_Interest/Region_Of_Interest">
																<div class="masque">
																	<xsl:attribute name="id">qcroi<xsl:value-of select="position()"/>
																	</xsl:attribute>
																	<table class="ROI" border="0" cellspacing="0" cellpadding="0" width="100%" height="100%">
																		<xsl:attribute name="title">ROI <xsl:value-of select="position()"/>
																		</xsl:attribute>
																		<tr>
																			<td></td>
																		</tr>
																	</table>
																</div>
															</xsl:for-each>
												
														</div>
													</td>
													<td width="80%">
														<table border="0" cellpadding="2" cellspacing="5">
															<xsl:if test="//GEOMETRIC_PROCESSING">
																<tr>
																	<td width="8"></td>
																	<td bgcolor="#00607a" width="70">
																		<p class="intitule2"> Type</p>
																	</td>
																	<td width="20"></td>
																	<td>
																		<p class="intitule">
																			<xsl:value-of select="//Production/PRODUCT_INFO"/>
																		</p>
																	</td>
																</tr>
															</xsl:if>
															<xsl:if test="//DATASET_NAME">
																<tr>
																	<td width="8"></td>
																	<td bgcolor="#00607a" width="70">
																		<p class="intitule2"> Layer</p>
																	</td>
																	<td width="20"></td>
																	<td>
																		<p class="intitule">
																			<xsl:value-of select="//Dataset_Id/DATASET_NAME"/>
																		</p>
																	</td>
																</tr>
															</xsl:if>
															<xsl:if test="//METADATA_FORMAT">
																<tr>
																	<td width="8"></td>
																	<td bgcolor="#00607a" width="70">
																		<p class="intitule2"> Format</p>
																	</td>
																	<td width="20"></td>
																	<td>
																		<p class="intitule">
																			<xsl:value-of select="//Metadata_Id/METADATA_FORMAT"/>
																		</p>
																	</td>
																</tr>
															</xsl:if>
															<xsl:if test="//DATA_FILE_FORMAT">
																<tr>
																	<td width="8"></td>
																	<td bgcolor="#00607a" width="70">
																		<p class="intitule2"> Raster</p>
																	</td>
																	<td width="20"></td>
																	<td>
																		<p class="intitule">
																			<xsl:value-of select="//Data_Access/DATA_FILE_FORMAT"/>
																		</p>
																	</td>
																</tr>
															</xsl:if>
														</table>
													</td>
													<td>
														<div align="right">
															<p>
																<img WIDTH="120" src="../LOGO.JPG" border="0" align="middle"/>
															</p>
														</div>
													</td>
												</tr>
											</table>
										</td>
									</tr>
								</table>
							</td>
						</tr>
					</table>
			
					<br/>
					<div id="divGenInfo" class="affiche">
						<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
							<tr>
								<td valign="middle" align="center">
									<br/>
									<table border="0" cellpadding="0" cellspacing="0" width="95%">
										<tr>
											<td valign="top">
												<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
													<tr>
														<td>
															<p>
																<span class="entete">General Information</span>
															</p>
														</td>
													</tr>
												</table>
												<br/>
												<table border="0" cellpadding="0" cellspacing="0" width="100%">
													<tr>
														<td valign="top">
															<p class="intitule">
																<b>Map Name</b>
															</p>
														</td>
														<td>
															<p class="description">
																<xsl:value-of select="//Dataset_Id/DATASET_NAME"/>
																<br/>
																<br/>
															</p>
														</td>
													</tr>
													<xsl:if test="//GEOMETRIC_PROCESSING">
														<tr>
															<td valign="top">
																<p class="intitule">
																	<b>Geometric Processing Level</b>
																</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//GEOMETRIC_PROCESSING"/>
																	<br/>
																	<br/>
																</p>
															</td>
														</tr>
													</xsl:if>
													<xsl:if test="//RADIOMETRIC_PROCESSING">
														<tr>
															<td valign="top">
																<p class="intitule">
																	<b>Radiometric Processing Level</b>
																</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//RADIOMETRIC_PROCESSING"/>
																	<br/>
																	<br/>
																</p>
															</td>
														</tr>
													</xsl:if>
												</table>
											</td>
										</tr>
									</table>
									<br/>
								</td>
							</tr>
						</table>
					</div>	<!-- /GenInfo -->
		
					<a name="description"></a>
					<div id="divImgDim" class="affiche">
						<br/>
						<xsl:if test="//Raster_Dimensions">
							<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
								<tr>
									<td valign="middle" align="center">
										<br/>
										<table border="0" cellpadding="0" cellspacing="0" width="95%">
											<tr>
												<td valign="top">
													<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
														<tr>
															<td>
																<p>
																	<span class="entete">Image dimensions</span>
																</p>
															</td>
														</tr>
													</table>
													<br/>
													<table border="0" cellpadding="0" cellspacing="5" width="100%">
														<xsl:if test="//NCOLS">
															<tr>
																<td valign="top" width="300">
																	<p class="intitule">Number of columns</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//NCOLS"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//NROWS">
															<tr>
																<td valign="top" width="300">
																	<p class="intitule">Number of rows</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//NROWS"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//NBANDS">
															<tr>
																<td valign="top" width="300">
																	<p class="intitule">Number of spectral bands</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//NBANDS"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
													</table>
												</td>
											</tr>
										</table>
										<br/>
									</td>
								</tr>
							</table>
						</xsl:if>
					</div> <!-- /divImgDim -->
		
					<a name="sourceinfo"></a>
					<div id="divDtstFrm" class="affiche">
						<xsl:if test="//Dataset_Frame">
							<br/>
							<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
								<tr>
									<td valign="middle" align="center">
										<br/>
										<table border="0" cellpadding="0" cellspacing="0" width="95%">
											<tr>
												<td valign="top">
													<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
														<tr>
															<td>
																<p>
																	<span class="entete">Dataset framing</span>
																</p>
															</td>
														</tr>
													</table>
													<br/>
													<table border="0" cellpadding="0" cellspacing="2" width="100%">
														<tr>
															<td valign="top" width="33%">
																<p class="intitule">Vertice</p>
															</td>
															<td valign="top" width="33%">
																<p class="description">
																	<span class="intitule">Longitude </span>(DEG)</p>
															</td>
															<td>
																<p class="description">
																	<span class="intitule">Latitude </span>(DEG)</p>
															</td>
															<td>
																<p class="description">
																	<span class="intitule">Row </span>
																</p>
															</td>
															<td>
																<p class="description">
																	<span class="intitule">Col </span>
																	<br/>
																</p>
															</td>
														</tr>
														<xsl:for-each select="//Dataset_Frame/Vertex">
															<tr>
																<td valign="top" width="33%">
																	<p class="description">#<xsl:value-of select="position()"/>
																	</p>
																</td>
																<td valign="top" width="33%">
																	<p class="description">
																		<xsl:value-of select="FRAME_LON"/>
																	</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="FRAME_LAT"/>
																	</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="FRAME_ROW"/>
																	</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="FRAME_COL"/>
																	</p>
																</td>
															</tr>
														</xsl:for-each>
														<xsl:if test="//Regions_Of_Interest">
															<tr>
									
																<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
																	<tr>
																		<td>
																			<p class="entete">
																				<b>Scene extracts</b>
																			</p>
																		</td>
																	</tr>
																</table>
																<table border="0" cellpadding="0" cellspacing="2" width="100%">
																	<tr>
																		<td valign="top" width="33%">
																			<p class="intitule">Region Of Interest</p>
																		</td>
																		<td valign="top" width="33%">
																			<p class="description">
																				<span class="intitule">Upper Left</span>
																			</p>
																		</td>
																		<td>
																			<p class="description">
																				<span class="intitule">Lower Right </span>
																			</p>
																		</td>
																	</tr>
																	<xsl:for-each select="//Regions_Of_Interest/Region_Of_Interest">
																		<tr>
																			<td valign="top" width="33%">
																				<p class="description">#<xsl:value-of select="position()"/>
																				</p>
																			</td>
																			<td valign="top" width="33%">
																				<p class="description">Row: <xsl:value-of select="ROW_MIN"/>, Col: <xsl:value-of select="COL_MIN"/>
																				</p>
																			</td>
																			<td>
																				<p class="description">Row: <xsl:value-of select="ROW_MAX"/>, Col: <xsl:value-of select="COL_MAX"/>
																				</p>
																			</td>
																		</tr>
																	</xsl:for-each>
																</table>
															</tr>
														</xsl:if>
													</table>
												</td>
											</tr>
										</table>
										<br/>
									</td>
								</tr>
							</table>
						</xsl:if>
					</div> <!-- /divDtstFrm -->
		
					<div id="divSources" class="masque">
						<br/>
						<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
							<tr>
								<td valign="middle" align="center">
									<br/>
									<table border="0" cellpadding="0" cellspacing="0" width="95%">
										<tr>
											<td valign="top">
												<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
													<tr>
														<td>
															<p>
																<span class="entete">Dataset sources</span>
															</p>
														</td>
													</tr>
												</table>
												<br/>
												<table border="0" cellpadding="0" cellspacing="5" width="100%">
										
													<xsl:for-each select="//Source_Information">
														<tr>
															<td valign="top" width="50">
																<p class="intitule">
																	<a title="show / hide details">
																		<xsl:attribute name="href">javascript:details("div<xsl:value-of select="generate-id()"/>")</xsl:attribute>
																		<xsl:value-of select="Scene_Source/MISSION"/>
																	</a>
																</p>
															</td>
															<td class="description">
																<xsl:value-of select="SOURCE_ID"/>
															</td>
														</tr>
														<tr>
															<td valign="top"></td>
															<td>
																<div class="masque">
																	<xsl:attribute name="id">div<xsl:value-of select="generate-id()"/>
																	</xsl:attribute>
																	<table border="0" cellspacing="5" width="100%">
																		<tr valign="top">
																			<td width="200">
																				<table border="0">
																					<tr>
																						<td class="intitule" width="50">ID</td>
																						<td class="description">
																							<xsl:value-of select="SOURCE_ID"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="50">K_J</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/GRID_REFERENCE"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="50">SAT</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/SHIFT_VALUE"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="50">DATE</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/IMAGING_DATE"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="50">TIME</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/IMAGING_TIME"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">INSTRUMENT</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/INSTRUMENT"/>
																							<xsl:value-of select="//Source_Information/Scene_Source/INSTRUMENT_INDEX"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">SENSOR</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/SENSOR_CODE"/>
																						</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">INCIDENCE_ANGLE</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/INCIDENCE_ANGLE"/> (<xsl:value-of select="Scene_Source/INCIDENCE_ANGLE/@unit"/>)</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">VIEWING_ANGLE</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/VIEWING_ANGLE"/> (<xsl:value-of select="Scene_Source/INCIDENCE_ANGLE/@unit"/>)</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">SUN_AZIMUTH</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/SUN_AZIMUTH"/> (<xsl:value-of select="Scene_Source/INCIDENCE_ANGLE/@unit"/>)</td>
																					</tr>
																					<tr>
																						<td class="intitule" width="65">SUN_ELEVATION</td>
																						<td class="description">
																							<xsl:value-of select="Scene_Source/SUN_ELEVATION"/> (<xsl:value-of select="Scene_Source/INCIDENCE_ANGLE/@unit"/>)</td>
																					</tr>
																				</table>
																			</td>
																			<td width="250">
																				<table border="0">
																					<tr>
																						<td class="intitule" width="65">BAND DESCRIPTION</td>
																						<td class="description">
																							<xsl:for-each select="//Spectral_Band_Info">BAND <xsl:value-of select="BAND_INDEX"/>
																								<BR/>
																								<FONT FACE="Trebuchet MS" STYLE="font-size: 8 pt;">
	  	    														Type : <xsl:value-of select="BAND_DESCRIPTION"/>
																									<BR/>
																									<xsl:if test="//Spectral_Band_Info/PHYSICAL_UNIT">
	  	      															Unit : <xsl:value-of select="PHYSICAL_UNIT"/>
																										<BR/>
																									</xsl:if>
																									<xsl:if test="//Spectral_Band_Info/PHYSICAL_GAIN">
	  	    													  Gain : <xsl:value-of select="PHYSICAL_GAIN"/>
																										<BR/>
																									</xsl:if>
																									<xsl:if test="//Spectral_Band_Info/PHYSICAL_BIAS">
	  	      												Bias : <xsl:value-of select="PHYSICAL_BIAS"/>
																										<BR/>
																									</xsl:if>
																								</FONT>
																							</xsl:for-each>
																						</td>
																					</tr>
																				</table>
																			</td>
																		</tr>
																	</table>
																</div>
															</td>
														</tr>
													</xsl:for-each>
												</table>
											</td>
										</tr>
									</table>
									<br/>
								</td>
							</tr>
						</table>
					</div> <!-- /divSources -->
			
					<div id="divCRS" class="masque">
						<br/>
						<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
							<tr>
								<td valign="middle" align="center">
									<br/>
									<table border="0" cellpadding="0" cellspacing="0" width="95%">
										<tr>
											<td valign="top">
												<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
													<tr>
														<td>
															<p>
																<span class="entete">Coordinate Reference System</span>
															</p>
														</td>
													</tr>
												</table>
												<br/>
												<table border="0" cellpadding="0" cellspacing="5" width="100%">
													<tr>
														<td valign="top">
															<p class="intitule">
																<font color="#A9AB87">Horizontal Coordinate System</font>
															</p>
														</td>
														<td></td>
													</tr>
													<xsl:if test="//GEO_TABLES">
														<tr>
															<td valign="top">
																<p class="intitule">Geocoding tables identification</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//GEO_TABLES"/>(<xsl:value-of select="//GEO_TABLES/@version"/>)</p>
															</td>
														</tr>
													</xsl:if>
													<xsl:if test="//HORIZONTAL_CS_TYPE">
														<tr>
															<td valign="top">
																<p class="intitule">Horizontal Coordinate System type</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//HORIZONTAL_CS_TYPE"/>
																</p>
															</td>
														</tr>
													</xsl:if>
													<xsl:if test="//HORIZONTAL_CS_NAME">
														<tr>
															<td valign="top">
																<p class="intitule">Horizontal coordinate system identification name</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//HORIZONTAL_CS_NAME"/>
																</p>
															</td>
														</tr>
													</xsl:if>
													<tr>
														<td valign="top">
															<p class="intitule">
																<font color="#A9AB87">Geographic Coordinate System</font>
															</p>
														</td>
														<td>
															<p class="description"></p>
														</td>
													</tr>
													<xsl:if test="//GEOGRAPHIC_CS_NAME">
														<tr>
															<td valign="top">
																<p class="intitule">Geographic Coordinate System</p>
															</td>
															<td>
																<p class="description">
																	<xsl:value-of select="//GEOGRAPHIC_CS_NAME"/>
																</p>
															</td>
														</tr>
													</xsl:if>
												</table>
											</td>
										</tr>
									</table>
									<br/>
								</td>
							</tr>
						</table>
					</div> <!-- /divCRS -->
			
					<div id="divGEO" class="masque">
						<br/>
						<xsl:if test="//ULXMAP">
			
							<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
								<tr>
									<td valign="middle" align="center">
										<br/>
										<table border="0" cellpadding="0" cellspacing="0" width="95%">
											<tr>
												<td valign="top">
													<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
														<tr>
															<td>
																<p>
																	<span class="entete">Geoposition</span>
																</p>
															</td>
														</tr>
													</table>
													<br/>
													<table border="0" cellpadding="0" cellspacing="5" width="100%">
														<xsl:if test="//ULXMAP">
															<tr>
																<td valign="top">
																	<p class="intitule">Upper - Left Map X Coordinate</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//ULXMAP"/>
																		<xsl:value-of select="//ULXMAP/@unit"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//ULYMAP">
															<tr>
																<td valign="top">
																	<p class="intitule">Upper - Left Map Y Coordinate</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//ULYMAP"/>
																		<xsl:value-of select="//ULXMAP/@unit"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//XDIM">
															<tr>
																<td valign="top">
																	<p class="intitule">Image sampling</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//XDIM"/> x <xsl:value-of select="//YDIM"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
													</table>
												</td>
											</tr>
										</table>
										<br/>
									</td>
								</tr>
							</table>
						</xsl:if>
					</div> <!-- /divGEO -->
			
					<div id="divProd" class="affiche">
						<br/>
						<xsl:if test="//Production">
							<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
								<tr>
									<td valign="middle" align="center">
										<br/>
										<table border="0" cellpadding="0" cellspacing="0" width="95%">
											<tr>
												<td valign="top">
													<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
														<tr>
															<td>
																<p>
																	<span class="entete">Production</span>
																</p>
															</td>
														</tr>
													</table>
													<br/>
													<table border="0" cellpadding="0" cellspacing="5" width="100%">
														<xsl:if test="//DATASET_PRODUCTION_DATE">
															<tr>
																<td valign="top">
																	<p class="intitule">Production Date</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//DATASET_PRODUCTION_DATE"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//JOB_ID">
															<tr>
																<td valign="top">
																	<p class="intitule">Job identification</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//JOB_ID"/>
																		<br/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//PRODUCT_TYPE">
															<tr>
																<td valign="top">
																	<p class="intitule">Product type identification</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//PRODUCT_TYPE"/>
																		<br/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//DATASET_PRODUCER_NAME">
															<tr>
																<td valign="top">
																	<p class="intitule">Dataset Producer Identification</p>
																</td>
																<td>
																	<p class="description">
																		<xsl:value-of select="//DATASET_PRODUCER_NAME"/>
																	</p>
																</td>
															</tr>
														</xsl:if>
														<xsl:if test="//DATASET_PRODUCER_URL">
															<tr>
																<td valign="top">
																	<p class="intitule">Producer link</p>
																</td>
																<td>
																	<p class="description">
																		<a target="_blank">
																			<xsl:attribute name="href">
																				<xsl:value-of select="//DATASET_PRODUCER_URL/@href"/>
																			</xsl:attribute>
																			<xsl:value-of select="//DATASET_PRODUCER_URL/@href"/>
																		</a>
																	</p>
																</td>
															</tr>
														</xsl:if>
													</table>
												</td>
											</tr>
										</table>
										<br/>
									</td>
								</tr>
							</table>
						</xsl:if>
					</div>
					<a name="quicklook"></a>
					<div id="divQuicklook" class="masque">
						<br/>
						<xsl:if test="//DATASET_QL_PATH/@href">
							<table border="3" bordercolor="#A9AB87" cellpadding="0" cellspacing="0" width="80%" bgcolor="#eeeeee">
								<tr>
									<td valign="middle" align="center">
										<br/>
										<table border="0" cellpadding="0" cellspacing="0" width="95%">
											<tr>
												<td valign="top">
													<div align="center">
														<table border="0" cellpadding="0" cellspacing="2" width="100%" bgcolor="#A9AB87">
															<tr>
																<td>
																	<p>
																		<span class="entete">Quicklook</span>
																	</p>
																</td>
															</tr>
														</table>
														<br/>
														<img id="imgQl" width="500" height="500" border="0">
															<xsl:attribute name="src">
																<xsl:value-of select="//DATASET_QL_PATH/@href"/>
															</xsl:attribute>
														</img>
													</div>
													<xsl:for-each select="//Regions_Of_Interest/Region_Of_Interest">
														<div class="affiche">
															<xsl:attribute name="id">roi<xsl:value-of select="position()"/>
															</xsl:attribute>
															<xsl:attribute name="title">ROI <xsl:value-of select="position()"/>
															</xsl:attribute>
															<table class="ROI" border="0" cellspacing="0" cellpadding="0" width="100%" height="100%">
																<xsl:attribute name="title">ROI <xsl:value-of select="position()"/>
																</xsl:attribute>
																<tr>
																	<td></td>
																</tr>
															</table>
														</div>
													</xsl:for-each>
								
												</td>
											</tr>
										</table>
										<br/>
									</td>
								</tr>
							</table>
						</xsl:if>
					</div>
				</div>
			</body>
		</html>
	</xsl:template>
</xsl:stylesheet>
