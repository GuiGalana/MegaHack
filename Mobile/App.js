import React from 'react';
import MapView, { Marker } from 'react-native-maps';
import { StyleSheet, Text, View, Dimensions } from 'react-native';

var i = 0
var colors = ['#00FF00','#FF0000','#FFFF00','#00FF00','#FF0000','#FFFF00','#00FF00','#FF0000','#FFFF00','#00FF00','#FF0000','#FFFF00']
var markerC = []
var m = 0
var markerImg = require('./images/barco_vermelinho.png');
var markerImg2 = require('./images/barco_verdinho.png');
var markerImg3 = require('./images/barco_amarelinho.png');

export default class App extends React.Component {
  constructor(props) {
    super(props);
    this.state = {
      reports: []
    }
  }
  render() {
	if (!this.state.reports){
		return (
		<div>Loading...</div>
		);
	}
    return (
      <View style={styles.container}>
        <MapView 
			style={styles.mapStyle} 
			initialRegion={{
			latitude: -23.9207,
			longitude: -46.3394,
			latitudeDelta: 0.3,
			longitudeDelta: 0.3,
			}}
			>
			{this.mapMarkers()}
		</MapView>
      </View>
    );
  }


componentDidMount() {
	 this.getInfo()
	 this.interval = setInterval(this.getInfo, 5000);
	 //this.setState({reports: teste2[i]})
	
	 
}

componentWillUnmount() {
    // Clear the interval right before component unmount
    clearInterval(this.interval);
}

getInfo = () => {
	var teste2 =[[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9207,
	   "lon":-46.3394,
	   "vel":4,
	   "spin":85,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9185,
	   "lon":-46.3486,
	   "vel":4,
	   "spin":80,
	   "estado":"Manobra",
		"color":'#FFFF00'}],
	 [{
	   "id":1,
	   "equip":1,
	   "lat":-23.9204,
	   "lon":-46.3406,
	   "vel":3,
	   "spin":90,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9191,
	   "lon":-46.3489,
	   "vel":3,
	   "spin":90,
	   "estado":"Efetivo",
	   "color":'#00FF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9212,
	   "lon":-46.3399,
	   "vel":3,
	   "spin":90,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9196,
	   "lon":-46.3492,
	   "vel":4,
	   "spin":105,
	   "estado":"Efetivo",
	   "color":'#00FF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9209,
	   "lon":-46.3413,
	   "vel":4,
	   "spin":98,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.92,
	   "lon":-46.3492,
	   "vel":5,
	   "spin":90,
	   "estado":"Efetivo",
	   "color":'#00FF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9205,
	   "lon":-46.3422,
	   "vel":4,
	   "spin":90,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.92,
	   "lon":-46.3492,
	   "vel":4,
	   "spin":85,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9203,
	   "lon":-46.3428,
	   "vel":5,
	   "spin":95,
	   "estado":"Efetivo",
	   "color":'green'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9204,
	   "lon":-46.3493,
	   "vel":2.5,
	   "spin":110,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9202,
	   "lon":-46.3435,
	   "vel":2,
	   "spin":95,
	   "estado":"Efetivo",
	   "color":'green'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9209,
	   "lon":-46.3494,
	   "vel":5,
	   "spin":100,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.92,
	   "lon":-46.3442,
	   "vel":8,
	   "spin":80,
	   "estado":"Efetivo",
	   "color":'green'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9212,
	   "lon":-46.3495,
	   "vel":5,
	   "spin":90,
	   "estado":"Manobra",
	   "color":'#FFFF00'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9198,
	   "lon":-46.3451,
	   "vel":10,
	   "spin":90,
	   "estado":"Efetivo",
	   "color":'green'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9212,
	   "lon":-46.3495,
	   "vel":0,
	   "spin":90,
	   "estado":"Parado",
	   "color":'#FF0000'
	 }],[{
	   "id":1,
	   "equip":1,
	   "lat":-23.9185,
	   "lon":-46.3478,
	   "vel":10,
	   "spin":95,
	   "estado":"Efetivo",
	   "color":'green'
	 },{
	   "id":2,
	   "equip":1,
	   "lat":-23.9212,
	   "lon":-46.3495,
	   "vel":0,
	   "spin":90,
	   "estado":"Parado",
	   "color":'#FF0000'
	 }]]
	 
	markerC = []
	fetch('https://enigmatic-reaches-55405.herokuapp.com/reports')
      .then(res => res.json())
      .then(data => {
        this.setState({ reports: teste2[i]})
		//this.setState({ reports: data.reports })
      })
      .catch(console.error)
	  
	  /* for (j = 0; teste2[i].length; j++){
		  if (teste2[i].estado === "E")
				markerC.push(1)
		  else if (teste2[i].estado==="M")
				markerC.push(2)
		  else
		        markerC.push(1)
	  } */
	  
	  i = i + 1
	  if ( i == 10)
		  i = 0
	 
	  
	  
}

chooseColor(data) {
	let ships;
	//console.log(data)
	//for (var j=0; j < data.length; j++){
		if (data === "Efetivo")
			ships = markerImg2
		else if (data == "Manobra")
			ships = markerImg3
		else
			ships = markerImg
	
	//console.log(ships)
	//console.log
	return ships
}

mapMarkers = () => {
    return this.state.reports.map((report) => 
	<Marker
      image={this.chooseColor(report.estado)}
	  key={report.id}
      coordinate={{ latitude: report.lat, longitude: report.lon }}
      //title={report.id}
	  //title={String(report.id)}
      //description={report.estado}
	  pinColor={report.color} 
	  
    >
	<MapView.Callout>
        <View style={{height: 80, width: 120}}>	
          <Text> Frota: {report.id} </Text>
		  <Text> Estado: {report.estado} </Text>
          <Text> Velocidade: {report.vel}</Text>
          <Text> Spin: {report.spin} </Text>
        </View>
    </MapView.Callout>
	
    </Marker >)
  }
  
 
  
}  
const styles = StyleSheet.create({
  container: {
    flex: 1,
    backgroundColor: '#fff',
    alignItems: 'center',
    justifyContent: 'center',
  },
  mapStyle: {
    width: Dimensions.get('window').width,
    height: Dimensions.get('window').height,
  },
});
