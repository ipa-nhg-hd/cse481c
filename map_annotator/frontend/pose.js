Pose = function(userActions, name) {
  var that = this;
  this.name = name;

  function handleGoTo() {
    var msg = new ROSLIB.Message({
      command: 'goto',
      name: name,
      updated_name: ''
    });
    userActions.publish(msg);
  }

  function handleDelete() {
    var msg = new ROSLIB.Message({
      command: 'delete',
      name: name,
      updated_name: ''
    });
    userActions.publish(msg);
  }

  this.render = function() {
    var node = document.createElement('div');
    var nameNode = document.createTextNode(name);
    node.appendChild(nameNode)

    var sendNode = document.createElement('input');
    sendNode.type = 'button';
    sendNode.value = 'Go to';
    sendNode.addEventListener('click', handleGoTo);
    node.appendChild(sendNode);

    var deleteNode = document.createElement('input');
    deleteNode.type = 'button';
    deleteNode.value = 'Delete';
    deleteNode.addEventListener('click', handleDelete);
    node.appendChild(deleteNode);
    return node;
  }
}
