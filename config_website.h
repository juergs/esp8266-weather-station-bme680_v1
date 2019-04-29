//--- spanned up website to set some cofigurations after having setup primary Wifi-configuration.
//--- 
const char WEB_PAGE_HEADER[] PROGMEM = "<html>\
<head>\
<title>ESP8266 Weather Station Color</title>\
<meta name='viewport' content='width=device-width'>\
<style type='text/css'>\
body{font-family:Arial;margin:0}\
.content{margin:10px}\
.r{text-align:right}\
td{vertical-align:top;}\
a{text-decoration:none;padding:10px;background:#38b5ad;color:white;display:block;width:auto;border-radius:5px;}\
input[type='text']{width:100%;}\
input[type='password']{width:100%;}\
input[type='submit']{border-radius:5px;font-size:medium;padding:5px;}\
.submit_green{padding:9px !important;width:100%;border-style:none;background:#38b5ad;color:white;text-align:left;}\
</style>\
</head><body>\
<div style='min-height:120px;background-color:#38b5ad;margin-bottom:20px'>\
<table><tr>\
  <td><br/><h3 style='margin:0'>ESP8266 Weather Station Color</h3>\
</tr></table>\
</div>\
<div class='content'><h4>{h} {n} {t}</h4>";
/*
 <small>ID: {id}<br/>MAC: {mac}<br/>IP: {ip} Gateway: {gw} Subnet: {sub}<br/>{fwt}: {fw}<br/>{ntp}</small></td>\
*/
const char WEB_ROOT_PAGE_CONTENT[] PROGMEM = "\
<a href='/config'>Konfiguration</a><br/>\
<a href='/reset'>Neustart</a><br/>\
";

/*
 <a href='/removeConfig'>Konfiguration löschen</a><br/>\
*/

const char WEB_PAGE_FOOTER[] PROGMEM = "<br/><br/><a href='/' style='display:inline;'>Zurück zur Startseite</a><br/><br/><br/>\
</div></body></html>\r\n";

const char WEB_PAGE_FOOTER_II[] PROGMEM = "<br/><br/><a href='/reset' style='display:inline;'>Wetterstation neu starten</a><br/><br/><br/>\
</div></body></html>\r\n";

const char WEB_PAGE_FOOTER_III[] PROGMEM = "<br/><br/>\
</div></body></html>\r\n";

const char WEB_RESET_CONTENT[] PROGMEM = "<h3>{q}</h3>\
<table><tr><td><form method='POST' action'/reset'><input type='submit' formmethod='post' class='submit_green' name='submit' value='{b}'/></form></td><td><a href='/'>{c}</a></td></tr></table>\
";
