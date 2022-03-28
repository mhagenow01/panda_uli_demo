import React, { Component } from 'react';
// import ROS3D from 'ros3d';
import roslib from 'roslib';
export default class Stream extends Component {
	constructor(props) {
		super(props);
		this.state = { imagedata: '' };
		
	}
	//

	render() {

		return (
			<img
				id='livestream'
				alt='Authoring-Robot Stream'
				style={{
					width: '1200px',
					//height: '1080px',
					//height: window.innerHeight,
					objectFix: 'cover',
					zIndex: 1,
				}}
			/>
		);
	}
}
